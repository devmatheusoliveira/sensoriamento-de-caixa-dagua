Final agora foi
//==============================================================================
// PROJETO 5-CP: Sistema para enchimento de caixa d'agua (HC-SR04) - V3
// MSP430G2553 - HC-SR04 + 8 LEDs (P2.0-P2.7) + Bomba + UART TX (USCI_A0)
//
// UM UNICO TIMER (Timer0_A), alternando entre dois modos:
//   1) MODO PWM (UP + OUTMOD_3 Set/Reset): gera por hardware o pulso de
//      Trigger de 15 us em P1.6 (TA0.1), no fim de cada periodo de 20 ms.
//   2) MODO CAPTURA (continuo, CCR0 com entrada CCI0A = P1.1): mede a
//      largura do Echo (RTT), com captura nas duas bordas.
//
// IMPORTANTE (limitacao de hardware do G2553 de 20 pinos):
//   As unicas entradas de captura EXTERNAS do Timer0_A sao P1.1 (CCI0A) e
//   P1.2 (CCI1A). Como P1.2 e o TX da UART (obrigatorio no projeto), o Echo
//   PRECISA ficar em P1.1. O RX da UART e sacrificado (o projeto so envia
//   dados ao PC, nunca recebe).
//
// Mapa de pinos:
//   P1.0        = Bomba (ponte H)
//   P1.1        = ECHO do HC-SR04 (TA0 CCI0A - captura) << com divisor de tensao!
//   P1.2        = UART TX (USCI_A0, 9600 bps)
//   P1.6        = TRIGGER do HC-SR04 (TA0.1 - saida PWM)
//   P2.0..P2.7  = 8 LEDs (barra de volume, porta 2 completa)
//
// SE ESTIVER USANDO A LAUNCHPAD:
//   - Remova o jumper RXD (P1.1) da ponte de jumpers, senao o emulador
//     "briga" com o sinal do Echo nesse pino. Mantenha o jumper TXD (P1.2).
//   - Remova o jumper do LED verde (P1.6) para nao carregar o Trigger.
//==============================================================================
#include <msp430.h>

#define QTDE_AMOSTRAS 32            // Janela movel de 32 amostras
#define PERIODO_TRIGGER 20000       // Periodo do PWM do Trigger: 20 ms
#define LARGURA_TRIGGER 15          // Pulso de Trigger de 15 us (>= 10 us)
#define TIMEOUT_ECO 60000           // ~60 ms sem Echo -> reinicia o ciclo
#define LIMITE_TIMEOUTS 25          // ~2 s sem eco -> aviso na serial
#define UART_DIVISOR 25
#define LED_DIVISOR 15

// Fases do Timer0 (o MESMO timer alterna entre PWM e Captura)
#define FASE_TRIGGER 0              // Timer0 em modo PWM gerando o Trigger
#define FASE_CAPTURA 1              // Timer0 reconfigurado em modo Captura (Echo)

void ini_uCon(void);
void ini_P1_P2(void);
void ini_UART(void);
void ini_Timer0_Modo_PWM_Trigger(void);

void atualizar_leds(float volume);
void controlar_bomba(float volume);
void uart_tx_char(char c);
void uart_tx_str(const char* s);
void uart_tx_float(float val);
float calcularVolume(float media_cm);

// Variaveis compartilhadas com as ISRs (interrupcoes)
volatile unsigned char fase = FASE_TRIGGER;
volatile unsigned char subida_ok = 0;   // So aceita borda de descida apos uma subida valida
volatile unsigned char contTimeouts = 0;
volatile unsigned int t_subida = 0;
volatile unsigned int amostrasLargura[QTDE_AMOSTRAS];
volatile unsigned long somaLarguras = 0;
volatile unsigned int indexAtual = 0;
volatile unsigned char bufferCheio = 0;
volatile unsigned char nova_medida = 0;

// Escala do reservatorio (0 a 510 ml)
float raio = 28.5f/10.0f;
float alturaTotal = 180.0f;
float volumeMinimo = 115.0f;
float volumeMaximo = 430.0f;
float volumeTotalCap = 510.0f;

void main(void) {
  float mediaAtual;
  float volumeAtual;
  unsigned long somaCopia;
  unsigned char contUart = 0;

  ini_uCon();
  ini_P1_P2();
  ini_UART();

  __enable_interrupt();           // Habilita as interrupcoes globais

  ini_Timer0_Modo_PWM_Trigger();  // Dispara o primeiro ciclo de medicao

  uart_tx_str("Sistema Iniciado e Monitorando...\r\n");

  do {
    // Diagnostico: eco nao esta chegando (fiacao/divisor/alimentacao)
    if (contTimeouts >= LIMITE_TIMEOUTS) {
      contTimeouts = 0;
      uart_tx_str("AVISO: Echo nao detectado!\r\n");
    }

    if (nova_medida) {
      nova_medida = 0;

      // Copia atomica segura da variavel de 32 bits (evita corrupcao pela ISR)
      __disable_interrupt();
      somaCopia = somaLarguras;
      __enable_interrupt();

      mediaAtual = (float)somaCopia / 1856.0f; // RTT(us) -> cm
      volumeAtual = calcularVolume(mediaAtual);

      if (bufferCheio) {
        atualizar_leds(volumeAtual);
        controlar_bomba(volumeAtual);
      }

      contUart++;
      if (contUart >= UART_DIVISOR) {
        contUart = 0;
        if (!bufferCheio) {
          uart_tx_str("Calibrando buffer inicial...\r\n");
        } else {
          uart_tx_str("Dist: ");
          uart_tx_float(mediaAtual);
          uart_tx_str(" cm | Vol: ");
          uart_tx_float(volumeAtual);
          uart_tx_str(" ml\r\n");
        }
      }
    }
  } while (1);
}

// Controle da bomba com histerese: liga no nivel minimo, desliga no maximo
void controlar_bomba(float volume) {
  if (volume < volumeMinimo) {
    // Liga a bomba
    P1OUT &= ~BIT0;
  } else if (volume >= volumeMaximo) {
    // Desliga a bomba
    P1OUT |= BIT0; 
  }
}

// Barra de 8 LEDs na porta 2 proporcional ao volume
void atualizar_leds(float volume) {
  unsigned char num_leds = 0;
  static unsigned char contPisca = 0;
  static unsigned char ledPiscaLigado = 0;

  if (volume > 0) {
    num_leds = (unsigned char)((volume / volumeMaximo) * 8.0f);
  }
  if (num_leds > 8) num_leds = 8;

  switch (num_leds) {
    case 0:
      contPisca++;
      if (contPisca >= LED_DIVISOR) {
        contPisca = 0;
        ledPiscaLigado = !ledPiscaLigado;
      }
      P2OUT = ledPiscaLigado ? BIT0 : 0;
      break;
    case 1: P2OUT = BIT0; break;
    case 2: P2OUT = BIT0 | BIT2; break;
    case 3: P2OUT = BIT0 | BIT2 | BIT3; break;
    case 4: P2OUT = BIT0 | BIT2 | BIT3 | BIT4; break;
    case 5: P2OUT = BIT0 | BIT2 | BIT3 | BIT4 | BIT5; break;
    case 6: P2OUT = BIT0 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6; break;
    case 7: P2OUT = BIT0 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6 | BIT7; break;
    case 8: P2OUT = 0xFF; break;
    default: break;
  }
}

// Converte a distancia media (cm) em volume (ml)
float calcularVolume(float media_cm) {
  float alturaAgua;

  alturaAgua = (alturaTotal / 10.0f) - media_cm;
  if (alturaAgua < 0) alturaAgua = 0;

  return 3.1415f * raio * raio * alturaAgua;
}

void ini_P1_P2(void) {
  // ----- Porta 1 -----
  P1DIR = 0xFF;
  P1DIR &= ~BIT1;              // P1.1 = ECHO (entrada de captura CCI0A)
  P1OUT = 0;

  P1SEL |= BIT2;               // P1.2 = UART TX (USCI_A0)
  P1SEL2 |= BIT2;

  P1SEL |= BIT1;               // P1.1 = TA0.0 / CCI0A (captura do Echo)
  P1SEL2 &= ~BIT1;             //   (P1SEL=1 e P1SEL2=0 seleciona a funcao Timer)

  P1SEL |= BIT6;               // P1.6 = TA0.1 (saida PWM do Trigger)
  P1SEL2 &= ~BIT6;

  // ----- Porta 2: 8 LEDs (porta completa) -----
  P2DIR = 0xFF;
  P2OUT = 0;
  P2SEL = 0;                   // Tudo GPIO, inclusive P2.6/P2.7 (XIN/XOUT)
  P2SEL2 = 0;
}

//------------------------------------------------------------------------------
// FASE 1: Timer0 em MODO PWM (UP + OUTMOD_3 Set/Reset)
// O pulso de Trigger de 15 us e gerado 100% por hardware no P1.6 (TA0.1),
// nos ultimos 15 us do periodo de 20 ms:
//   - EQU1 (CCR1 = periodo - 15): a saida SOBE  (inicio do pulso)
//   - EQU0 (CCR0 = periodo):      a saida DESCE (fim do pulso) + interrupcao
// Na interrupcao de CCR0 (fim do pulso), o MESMO timer e reconfigurado
// para o modo Captura.
//------------------------------------------------------------------------------
void ini_Timer0_Modo_PWM_Trigger(void) {
  fase = FASE_TRIGGER;
  TA0CTL = TASSEL_2 | MC_0 | TACLR;   // Para o timer para reconfigurar com seguranca
  TA0CCTL1 = OUTMOD_0;                // Saida do Trigger em nivel baixo (OUT = 0)
  TA0CCTL0 = 0;                       // CCR0 volta a ser COMPARACAO (limpa CAP e flags)
  TA0CCR0 = PERIODO_TRIGGER - 1;      // Periodo de 20 ms
  TA0CCR1 = PERIODO_TRIGGER - 1 - LARGURA_TRIGGER; // Pulso nos ultimos 15 us
  TA0CCTL1 = OUTMOD_3;                // PWM Set/Reset por hardware
  TA0CCTL0 = CCIE;                    // Interrupcao no fim do periodo (= fim do pulso)
  TA0CTL = TASSEL_2 | MC_1 | TACLR;   // Modo UP, SMCLK 1 MHz -> inicia
}

//------------------------------------------------------------------------------
// ISR do CCR0 do Timer0 - usada NAS DUAS FASES:
//   FASE_TRIGGER: comparacao no fim do pulso -> reconfigura p/ MODO CAPTURA
//   FASE_CAPTURA: captura das bordas do Echo em P1.1 (CCI0A) -> mede o RTT
//------------------------------------------------------------------------------
#pragma vector = TIMER0_A0_VECTOR
__interrupt void RTI_Timer0_CCR0(void) {
  unsigned int agora;
  unsigned int novaLargura;

  if (fase == FASE_TRIGGER) {
    // Fim do pulso de Trigger: RECONFIGURA o mesmo timer para CAPTURA
    fase = FASE_CAPTURA;
    subida_ok = 0;
    TA0CTL = TASSEL_2 | MC_0 | TACLR;             // Para o timer (troca de modo segura)
    TA0CCTL1 = OUTMOD_0;                          // Saida do Trigger fica em 0
    TA0CCTL0 = CM_3 | CCIS_0 | SCS | CAP | CCIE;  // Captura nas 2 bordas em CCI0A (P1.1)
    TA0CCTL0 &= ~CCIFG;                           // Descarta captura espuria da troca de modo
    TA0CCR1 = TIMEOUT_ECO;                        // Timeout caso o Echo nao chegue
    TA0CCTL1 = CCIE;                              // (OUTMOD_0/OUT=0, limpa flag pendente)
    TA0CTL = TASSEL_2 | MC_2 | TACLR;             // Modo continuo -> inicia a captura
  } else {
    // FASE_CAPTURA: uma borda do Echo foi capturada
    agora = TA0CCR0;

    if (TA0CCTL0 & COV) {
      TA0CCTL0 &= ~COV;                // Limpa flag de sobreposicao, se houver
    }

    if (TA0CCTL0 & CCI) {
      t_subida = agora;                // Borda de SUBIDA do Echo
      subida_ok = 1;
    } else if (subida_ok) {
      subida_ok = 0;
      novaLargura = agora - t_subida;  // Largura do pulso = RTT em us

      somaLarguras -= amostrasLargura[indexAtual]; // Remove amostra antiga da media movel
      somaLarguras += novaLargura;                 // Adiciona amostra nova
      amostrasLargura[indexAtual] = novaLargura;

      indexAtual++;
      if (indexAtual >= QTDE_AMOSTRAS) {
        indexAtual = 0;
        bufferCheio = 1;
      }
      nova_medida = 1;
      contTimeouts = 0;                // Eco valido: zera o diagnostico

      // Medida concluida: volta o MESMO timer ao modo PWM.
      // O proximo pulso de Trigger ocorre ~20 ms depois (fim do novo periodo).
      ini_Timer0_Modo_PWM_Trigger();
    }
  }
}

//------------------------------------------------------------------------------
// ISR do CCR1 do Timer0 (via TA0IV): TIMEOUT do Echo
// Se o Echo nao chegar em ~60 ms (sensor desconectado ou fora de alcance),
// reinicia o ciclo em vez de travar o sistema.
//------------------------------------------------------------------------------
#pragma vector = TIMER0_A1_VECTOR
__interrupt void RTI_Timer0_Timeout(void) {
  switch (TA0IV) {
    case TA0IV_TACCR1:
      if (fase == FASE_CAPTURA) {
        if (contTimeouts < 250) contTimeouts++;
        ini_Timer0_Modo_PWM_Trigger();
      }
      break;
    default:
      break;
  }
}

void ini_UART(void) {
  UCA0CTL1 |= UCSWRST;
  UCA0CTL1 |= UCSSEL_2;
  UCA0BR0 = 104;                  // 1 MHz / 9600 bps
  UCA0BR1 = 0;
  UCA0MCTL = UCBRS_1;
  UCA0CTL1 &= ~UCSWRST;
}

void uart_tx_char(char c) {
  while (!(IFG2 & UCA0TXIFG));
  UCA0TXBUF = c;
}

void uart_tx_str(const char* s) {
  int index = 0;
  char atual;

  do {
    atual = s[index];
    if (atual != '\0') {
      uart_tx_char(atual);
    }
    index++;
  } while (atual != '\0');
}

void uart_tx_float(float val) {
  char buf[6];
  int i = 0;
  int inteira, decimal, temp;

  inteira = (int)val;
  decimal = (int)((val - (float)inteira) * 100.0f);

  temp = inteira;
  if (temp == 0) {
    uart_tx_char('0');
  } else {
    while (temp > 0 && i < 5) {
      buf[i++] = (char)((temp % 10) + '0');
      temp /= 10;
    }
    while (i > 0) uart_tx_char(buf[--i]);
  }

  uart_tx_char(',');
  uart_tx_char((char)((decimal / 10) + '0'));
  uart_tx_char((char)((decimal % 10) + '0'));
}

void ini_uCon(void) {
  WDTCTL = WDTPW | WDTHOLD;

  BCSCTL1 = CALBC1_1MHZ;   // Primeiro BCSCTL1, depois DCOCTL
  DCOCTL = CALDCO_1MHZ;
  BCSCTL2 = 0;
}
