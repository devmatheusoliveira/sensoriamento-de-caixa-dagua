#include <msp430.h>

#define QTDE_AMOSTRAS 32            
#define PERIODO_TRIGGER 20000       
#define LARGURA_TRIGGER 15          
#define TIMEOUT_ECO 60000           
#define LIMITE_TIMEOUTS 25          
#define UART_DIVISOR 25
#define LED_DIVISOR 15

#define FASE_TRIGGER 0              
#define FASE_CAPTURA 1              

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

volatile unsigned char fase = FASE_TRIGGER;
volatile unsigned char subida_ok = 0;   
volatile unsigned char contTimeouts = 0;
volatile unsigned int t_subida = 0;
volatile unsigned int amostrasLargura[QTDE_AMOSTRAS];
volatile unsigned long somaLarguras = 0;
volatile unsigned int indexAtual = 0;
volatile unsigned char bufferCheio = 0;
volatile unsigned char nova_medida = 0;

float raio = 2.85f;
float alturaTotal = 180.0f;
float volumeMinimo = 115.0f;
float volumeMaximo = 430.0f;
float volumeTotalCap = 510.0f;

float ESCALA_LED = 8.0f / volumeMaximo

void main(void) {
  float mediaAtual;
  float volumeAtual;
  unsigned long somaCopia;
  unsigned char contUart = 0;

  ini_uCon();
  ini_P1_P2();
  ini_UART();

  __enable_interrupt();           

  ini_Timer0_Modo_PWM_Trigger();  

  uart_tx_str("Sistema Iniciado e Monitorando...\r\n");

  do {
    if (contTimeouts >= LIMITE_TIMEOUTS) {
      contTimeouts = 0;
      uart_tx_str("AVISO: Echo nao detectado!\r\n");
    }

    if (nova_medida) {
      nova_medida = 0;

      __disable_interrupt();
      somaCopia = somaLarguras;
      __enable_interrupt();

      mediaAtual = (float)somaCopia / 1856.0f; 
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

void controlar_bomba(float volume) {
  if (volume < volumeMinimo) {
    P1OUT &= ~BIT0;
  } else if (volume >= volumeMaximo) {
    P1OUT |= BIT0; 
  }
}

void atualizar_leds(float volume) {
  unsigned char num_leds = 0;
  static unsigned char contPisca = 0;
  static unsigned char ledPiscaLigado = 0;

  if (volume > 0) {
    num_leds = (unsigned char)(volume * ESCALA_LED);
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

float calcularVolume(float media_cm) {
  float alturaAgua;

  alturaAgua = (alturaTotal / 10.0f) - media_cm;
  if (alturaAgua < 0) alturaAgua = 0;

  return 3.1415f * raio * raio * alturaAgua;
}

void ini_P1_P2(void) {
  P1DIR = 0xFF;
  P1DIR &= ~BIT1;              
  P1OUT = 0;

  P1SEL |= BIT2;               
  P1SEL2 |= BIT2;

  P1SEL |= BIT1;               
  P1SEL2 &= ~BIT1;             

  P1SEL |= BIT6;               
  P1SEL2 &= ~BIT6;

  P2DIR = 0xFF;
  P2OUT = 0;
  P2SEL = 0;                   
  P2SEL2 = 0;
}

void ini_Timer0_Modo_PWM_Trigger(void) {
  fase = FASE_TRIGGER;
  TA0CTL = TASSEL_2 | MC_0 | TACLR;   
  TA0CCTL1 = OUTMOD_0;                
  TA0CCTL0 = 0;                       
  TA0CCR0 = PERIODO_TRIGGER - 1;      
  TA0CCR1 = PERIODO_TRIGGER - 1 - LARGURA_TRIGGER; 
  TA0CCTL1 = OUTMOD_3;                
  TA0CCTL0 = CCIE;                    
  TA0CTL = TASSEL_2 | MC_1 | TACLR;   
}

#pragma vector = TIMER0_A0_VECTOR
__interrupt void RTI_Timer0_CCR0(void) {
  unsigned int agora;
  unsigned int novaLargura;

  if (fase == FASE_TRIGGER) {
    fase = FASE_CAPTURA;
    subida_ok = 0;
    TA0CTL = TASSEL_2 | MC_0 | TACLR;             
    TA0CCTL1 = OUTMOD_0;                          
    TA0CCTL0 = CM_3 | CCIS_0 | SCS | CAP | CCIE;  
    TA0CCTL0 &= ~CCIFG;                           
    TA0CCR1 = TIMEOUT_ECO;                        
    TA0CCTL1 = CCIE;                              
    TA0CTL = TASSEL_2 | MC_2 | TACLR;             
  } else {
    agora = TA0CCR0;

    if (TA0CCTL0 & COV) {
      TA0CCTL0 &= ~COV;                
    }

    if (TA0CCTL0 & CCI) {
      t_subida = agora;                
      subida_ok = 1;
    } else if (subida_ok) {
      subida_ok = 0;
      novaLargura = agora - t_subida;  

      somaLarguras -= amostrasLargura[indexAtual]; 
      somaLarguras += novaLargura;                 
      amostrasLargura[indexAtual] = novaLargura;

      indexAtual++;
      if (indexAtual >= QTDE_AMOSTRAS) {
        indexAtual = 0;
        bufferCheio = 1;
      }
      nova_medida = 1;
      contTimeouts = 0;                

      ini_Timer0_Modo_PWM_Trigger();
    }
  }
}

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
  UCA0BR0 = 104;                  
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

  BCSCTL1 = CALBC1_1MHZ;   
  DCOCTL = CALDCO_1MHZ;
  BCSCTL2 = 0;
}
