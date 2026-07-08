
// agora foi

//==============================================================
// MSP430G2553 - HC-SR04 + 8 LEDs + Bomba (ponte H) + UART
//
// REQUISITO: media de 32 amostras (mantido).
// Rapidez obtida por:
//   1) MEDIA MOVEL com SOMA CORRIDA: a cada eco a amostra mais
//      antiga sai da soma e a nova entra. A media das 32 e
//      recalculada a CADA medicao, nao a cada 32.
//   2) Trigger a cada 20 ms (em vez de 60 ms). O eco de um
//      reservatorio de 20 cm dura ~1,2 ms; os 60 ms do datasheet
//      sao para 4 m. Janela completa: 32 x 20 ms = 0,64 s.
//
// Resultado: LEDs e bomba reavaliados a cada 20 ms, sempre com
// a media das 32 ultimas amostras.
//
// Mapa de pinos:
//   P1.1  - UCA0RXD (UART)
//   P1.2  - UCA0TXD (UART)
//   P1.6  - Trigger HC-SR04 (TA0.1, pulso 15 us a cada 20 ms)
//   P2.1  - Echo HC-SR04    (captura TA1.1)
//   P1.0  - Bomba / ponte H
//   LEDs  - P2.0, P2.2..P2.7, P1.7
//==============================================================
#include <msp430.h>

#define QTDE_AMOSTRAS   32      // requisito: media de 32 amostras
#define PERIODO_TRIGGER 19999   // 20 ms (SMCLK 1 MHz). Use 59999 p/ 60 ms
#define UART_DIVISOR    25      // imprime 1 a cada 25 medicoes (~0,5 s)

void ini_uCon(void);
void ini_P1_P2(void);
void ini_UART(void);
void ini_Timer0_PWM_Trigger(void);
void ini_Timer1_Captura_Echo(void);

void atualizar_leds(unsigned int dist_cm);
void controlar_bomba(float volume);
void uart_tx_char(char c);
void uart_tx_str(const char *s);
void uart_tx_float(float val);
float calcularVolume(float media_cm);

//---------------- variaveis compartilhadas com a ISR ----------
volatile unsigned int  t_subida = 0;
volatile unsigned int  amostrasLargura[QTDE_AMOSTRAS];
volatile unsigned long somaLarguras = 0;    // soma corrida das 32
volatile unsigned int  indexAtual = 0;
volatile unsigned char bufferCheio = 0;
volatile unsigned char nova_medida = 0;

//---------------- parametros do reservatorio ------------------
float raio        = 28.5f;      // mm
float alturaTotal = 200.0f;     // mm
float volumeMinimo = 115.0f;     // ml -> liga a bomba abaixo disso
float volumeMaximo = 460.0f;    // ml -> desliga a bomba a partir disso

void main(void)
{
    float mediaAtual;
    float volumeAtual;
    unsigned long somaCopia;
    unsigned char contUart = 0;

    ini_uCon();
    ini_P1_P2();
    ini_UART();
    ini_Timer0_PWM_Trigger();
    ini_Timer1_Captura_Echo();

    __enable_interrupt();

    uart_tx_str("Sistema iniciado\r\n");

    do {
        if (nova_medida) {              // chega a cada 20 ms
            nova_medida = 0;

            // copia atomica da soma corrida (ISR pode alterar)
            __disable_interrupt();
            somaCopia = somaLarguras;
            __enable_interrupt();

            // media das 32 ultimas amostras, em cm
            mediaAtual  = ((float)somaCopia / QTDE_AMOSTRAS) / 58.0f;
            volumeAtual = calcularVolume(mediaAtual);

            // --- acoes rapidas: rodam a cada medicao (20 ms) ---
            atualizar_leds((unsigned int)mediaAtual);
            controlar_bomba(volumeAtual);

            // --- acao lenta: UART so a cada ~0,5 s -------------
            contUart++;
            if (contUart >= UART_DIVISOR) {
                contUart = 0;
                uart_tx_str("Dist: ");
                uart_tx_float(mediaAtual);
                uart_tx_str(" cm | Vol: ");
                uart_tx_float(volumeAtual);
                uart_tx_str(" ml\r\n");
            }
        }
    } while (1);
}

//--------------------------------------------------------------
// Bomba com HISTERESE:
//   volume < volumeMinimo  -> LIGA  (P1.0 = 1)
//   volume >= volumeMaximo -> DESLIGA (P1.0 = 0)
//   entre os dois limites  -> mantem o estado atual
// Reavaliada a cada 20 ms, sempre com a media das 32 amostras.
//--------------------------------------------------------------
void controlar_bomba(float volume)
{
    if (volume < volumeMinimo) {
        P1OUT |= BIT0;                  // liga a bomba
    } else if (volume >= volumeMaximo) {
        P1OUT &= ~BIT0;                 // desliga a bomba
    }
    // entre min e max: nao mexe (evita liga/desliga oscilante)
}

//--------------------------------------------------------------
// Bargraph: quanto mais perto (reservatorio mais cheio),
// mais LEDs acesos. 1 LED a cada 5 cm; >= 40 cm -> 0 LEDs.
//--------------------------------------------------------------
void atualizar_leds(unsigned int dist_cm)
{
    static const unsigned char led_p2[8] = {BIT0, BIT2, BIT3, BIT4, BIT5, BIT6, BIT7, 0};
    static const unsigned char led_p1[8] = {0, 0, 0, 0, 0, 0, 0, BIT7};

    unsigned char n, i;
    unsigned char mask_p2 = 0, mask_p1 = 0;

    if (dist_cm >= 20)
        n = 0;
    else
        n = 8 - (dist_cm / 3);

    for (i = 0; i < n; i++) {
        mask_p2 |= led_p2[i];
        mask_p1 |= led_p1[i];
    }

    P2OUT = (P2OUT & ~(BIT0 + BIT2 + BIT3 + BIT4 + BIT5 + BIT6 + BIT7)) | mask_p2;
    P1OUT = (P1OUT & ~BIT7) | mask_p1;
}

//--------------------------------------------------------------
// ISR de captura do Echo (TA1.1 / P2.1).
// SOMA CORRIDA: na borda de descida, tira a amostra mais antiga
// da soma, coloca a nova no lugar e soma. Custo O(1) na ISR:
// a media das 32 fica pronta sem varrer o buffer.
//--------------------------------------------------------------
#pragma vector = TIMER1_A1_VECTOR
__interrupt void RTI_Captura_Echo(void)
{
    unsigned int novaLargura;

    switch (TA1IV) {

    case TA1IV_TACCR1:

        if (TA1CCTL1 & COV) {
            TA1CCTL1 &= ~COV;           // descarta captura sobreposta
        }

        if (TA1CCTL1 & CCI) {           // borda de subida
            t_subida = TA1CCR1;
        } else {                        // borda de descida
            novaLargura = TA1CCR1 - t_subida;

            somaLarguras -= amostrasLargura[indexAtual]; // sai a antiga
            somaLarguras += novaLargura;                 // entra a nova
            amostrasLargura[indexAtual] = novaLargura;

            indexAtual++;
            if (indexAtual >= QTDE_AMOSTRAS) {
                indexAtual = 0;
                bufferCheio = 1;        // a partir daqui a media e valida
            }
            if (bufferCheio) {
                nova_medida = 1;        // avisa o main A CADA amostra
            }
        }
        break;

    default:
        break;
    }
}

//--------------------------------------------------------------
// Volume em ml a partir da distancia media (cm).
// raio e alturaTotal em mm -> converte para cm.
//--------------------------------------------------------------
float calcularVolume(float media_cm)
{
    float alturaAtual;
    float raioCm;

    alturaAtual = (alturaTotal / 10.0f) - media_cm;
    if (alturaAtual < 0) {
        alturaAtual = 0;
    }

    raioCm = raio / 10.0f;
    return 3.14f * raioCm * raioCm * alturaAtual;   // ml (cm^3)
}

//--------------------------------------------------------------
// TA0: SMCLK 1 MHz, modo up, periodo PERIODO_TRIGGER.
// TA0.1 modo 7 (reset/set): pulso de 15 us no P1.6 (Trigger).
//--------------------------------------------------------------
void ini_Timer0_PWM_Trigger(void)
{
    TA0CTL   = TASSEL_2 | MC_1 | TACLR;
    TA0CCTL0 = 0;
    TA0CCTL1 = OUTMOD_7;
    TA0CCR0  = PERIODO_TRIGGER;         // 20 ms
    TA0CCR1  = 15;                      // 15 us
}

//--------------------------------------------------------------
// TA1: SMCLK 1 MHz, modo continuo.
// TA1.1 captura nas duas bordas, sincronizada, com interrupcao.
//--------------------------------------------------------------
void ini_Timer1_Captura_Echo(void)
{
    TA1CTL   = TASSEL_2 | MC_2 | TACLR;
    TA1CCTL1 = CM_3 | SCS | CAP | CCIE;
}

void ini_P1_P2(void)
{
    // ----- Porta 1 -----
    P1DIR  = 0xFF;
    P1DIR &= ~BIT1;                     // P1.1 = RXD (entrada)
    P1OUT  = 0;

    P1SEL  |= BIT1 | BIT2;              // UART (USCI_A0)
    P1SEL2 |= BIT1 | BIT2;

    P1SEL  |= BIT6;                     // P1.6 = TA0.1 (Trigger)
    P1SEL2 &= ~BIT6;

    // ----- Porta 2 -----
    P2DIR  = 0xFF;
    P2DIR &= ~BIT1;                     // P2.1 = Echo (entrada)
    P2OUT  = 0;

    P2SEL  &= ~(BIT6 | BIT7);           // XIN/XOUT como GPIO (LEDs)
    P2SEL2 &= ~(BIT6 | BIT7);

    P2SEL  |= BIT1;                     // P2.1 = TA1.1 (captura)
    P2SEL2 &= ~BIT1;
}

//--------------------------------------------------------------
// USCI_A0: 9600-8N1 com SMCLK = 1 MHz
//--------------------------------------------------------------
void ini_UART(void)
{
    UCA0CTL1 |= UCSWRST;
    UCA0CTL1 |= UCSSEL_2;
    UCA0BR0   = 104;
    UCA0BR1   = 0;
    UCA0MCTL  = UCBRS_1;
    UCA0CTL1 &= ~UCSWRST;
}

void uart_tx_char(char c)
{
    while (!(IFG2 & UCA0TXIFG));
    UCA0TXBUF = c;
}

void uart_tx_str(const char *s)
{
    while (*s)
        uart_tx_char(*s++);
}

void uart_tx_float(float val)
{
    char buf[12];
    int i = 0;
    long inteira, decimal, temp;

    if (val < 0) {
        uart_tx_char('-');
        val = -val;
    }

    inteira = (long)val;
    decimal = (long)((val - (float)inteira) * 100.0f + 0.5f);

    if (decimal >= 100) {
        inteira += 1;
        decimal -= 100;
    }

    temp = inteira;
    if (temp == 0) {
        uart_tx_char('0');
    } else {
        while (temp > 0) {
            buf[i++] = (char)((temp % 10) + '0');
            temp /= 10;
        }
        while (i > 0)
            uart_tx_char(buf[--i]);
    }

    uart_tx_char(',');
    uart_tx_char((char)((decimal / 10) + '0'));
    uart_tx_char((char)((decimal % 10) + '0'));
}

void ini_uCon(void)
{
    WDTCTL = WDTPW | WDTHOLD;

    BCSCTL1 = CALBC1_1MHZ;              // primeiro BCSCTL1, depois DCOCTL
    DCOCTL  = CALDCO_1MHZ;
    BCSCTL2 = 0;
}
