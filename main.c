//==============================================================
// MSP430G2553 - HC-SR04 + 8 LEDs + Ponte H + UART (terminal USB)
//
// Mapa de pinos:
//   P1.1  - UCA0RXD (UART, jumpers do LaunchPad em HW UART)
//   P1.2  - UCA0TXD (UART)
//   P1.6  - Trigger HC-SR04 (saida TA0.1, pulso de 15 us a cada 60 ms)
//   P2.1  - Echo HC-SR04    (entrada de captura TA1.1)
//   P1.0  - Sinal digital p/ ponte H (ativa se dist < DIST_LIMIAR)
//   LEDs  - P2.0, P2.2, P2.3, P2.4, P2.5, P2.6, P2.7, P1.7
//==============================================================
#include <msp430.h>

#define DIST_LIMIAR 10      // cm

void ini_uCon(void);
void ini_P1_P2(void);
void ini_UART(void);
void ini_Timer0_PWM_Trigger(void);
void ini_Timer1_Captura_Echo(void);
void atualizar_leds(unsigned int dist);
void uart_tx_char(char c);
void uart_tx_str(const char *s);
void uart_tx_num(unsigned int v);

volatile unsigned int t_subida    = 0;
volatile unsigned int largura     = 0;
volatile unsigned int distancia   = 0;
volatile unsigned char nova_medida = 0;

void main(void)
{
    ini_uCon();
    ini_P1_P2();
    ini_UART();
    ini_Timer0_PWM_Trigger();
    ini_Timer1_Captura_Echo();

    __enable_interrupt();

    uart_tx_str("HC-SR04 iniciado\r\n");

    do {
        if (nova_medida) {
            nova_medida = 0;

            atualizar_leds(distancia);

            uart_tx_str("Dist: ");
            uart_tx_num(distancia);
            uart_tx_str(" cm\r\n");
        }
    } while (1);
}

//--------------------------------------------------------------
// Bargraph: quanto mais perto, mais LEDs acesos.
// dist < 5 cm -> 8 LEDs ... dist >= 40 cm -> 0 LEDs (1 LED a cada 5 cm)
//--------------------------------------------------------------
void atualizar_leds(unsigned int dist)
{
    static const unsigned char led_p2[8] = {BIT0, BIT2, BIT3, BIT4, BIT5, BIT6, BIT7, 0};
    static const unsigned char led_p1[8] = {0, 0, 0, 0, 0, 0, 0, BIT7};

    unsigned char n, i;
    unsigned char mask_p2 = 0, mask_p1 = 0;

    if (dist >= 40)
        n = 0;
    else
        n = 8 - (dist / 5);

    for (i = 0; i < n; i++) {
        mask_p2 |= led_p2[i];
        mask_p1 |= led_p1[i];
    }

    P2OUT = (P2OUT & ~(BIT0 + BIT2 + BIT3 + BIT4 + BIT5 + BIT6 + BIT7)) | mask_p2;
    P1OUT = (P1OUT & ~BIT7) | mask_p1;
}

//--------------------------------------------------------------
// Captura do Echo em TA1.1 (P2.1): borda de subida guarda o
// instante inicial; borda de descida calcula a largura e a
// distancia (us / 58 = cm). Aciona a ponte H em P1.0.
//--------------------------------------------------------------
#pragma vector = TIMER1_A1_VECTOR
__interrupt void RTI_Captura_Echo(void)
{
    switch (TA1IV) {

    case TA1IV_TACCR1:

        if (TA1CCTL1 & COV) {
            TA1CCTL1 &= ~COV;           // descarta captura sobreposta
        }

        if (TA1CCTL1 & CCI) {           // borda de subida
            t_subida = TA1CCR1;
        } else {                        // borda de descida
            largura   = TA1CCR1 - t_subida;   // aritmetica sem sinal trata o estouro
            distancia = largura / 58;
            nova_medida = 1;

            if (distancia < DIST_LIMIAR) {
                P1OUT |= BIT0;          // ativa pino da ponte H
            } else {
                P1OUT &= ~BIT0;
            }
        }
        break;

    default:
        break;
    }
}

//--------------------------------------------------------------
// TA0: SMCLK (1 MHz), modo up, periodo 60 ms.
// TA0.1 em modo 7 (reset/set): sai um pulso de 15 us no P1.6.
//--------------------------------------------------------------
void ini_Timer0_PWM_Trigger(void)
{
    TA0CTL   = TASSEL_2 | MC_1 | TACLR;
    TA0CCTL0 = 0;
    TA0CCTL1 = OUTMOD_7;
    TA0CCR0  = 59999;                   // 60 ms
    TA0CCR1  = 15;                      // pulso de 15 us
}

//--------------------------------------------------------------
// TA1: SMCLK (1 MHz), modo continuo.
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

    P1SEL  |= BIT1 | BIT2;              // P1.1/P1.2 = UART (USCI_A0)
    P1SEL2 |= BIT1 | BIT2;

    P1SEL  |= BIT6;                     // P1.6 = TA0.1 (Trigger)
    P1SEL2 &= ~BIT6;

    // ----- Porta 2 -----
    P2DIR  = 0xFF;
    P2DIR &= ~BIT1;                     // P2.1 = Echo (entrada)
    P2OUT  = 0;

    P2SEL  &= ~(BIT6 | BIT7);           // libera XIN/XOUT como GPIO (LEDs)
    P2SEL2 &= ~(BIT6 | BIT7);

    P2SEL  |= BIT1;                     // P2.1 = TA1.1 (captura CCI1A)
    P2SEL2 &= ~BIT1;
}

//--------------------------------------------------------------
// USCI_A0: 9600-8N1 com SMCLK = 1 MHz
// BR = 1.000.000 / 9600 = 104,17 -> UCA0BR = 104, UCBRS = 1
//--------------------------------------------------------------
void ini_UART(void)
{
    UCA0CTL1 |= UCSWRST;                // mantem em reset durante config
    UCA0CTL1 |= UCSSEL_2;               // clock = SMCLK
    UCA0BR0   = 104;
    UCA0BR1   = 0;
    UCA0MCTL  = UCBRS_1;
    UCA0CTL1 &= ~UCSWRST;               // libera o modulo
}

void uart_tx_char(char c)
{
    while (!(IFG2 & UCA0TXIFG));        // espera buffer livre
    UCA0TXBUF = c;
}

void uart_tx_str(const char *s)
{
    while (*s)
        uart_tx_char(*s++);
}

void uart_tx_num(unsigned int v)
{
    char buf[6];
    unsigned char i = 0;

    if (v == 0) {
        uart_tx_char('0');
        return;
    }
    while (v > 0) {
        buf[i++] = '0' + (v % 10);
        v /= 10;
    }
    while (i > 0)
        uart_tx_char(buf[--i]);
}

void ini_uCon(void)
{
    WDTCTL = WDTPW | WDTHOLD;

    BCSCTL1 = CALBC1_1MHZ;              // primeiro BCSCTL1, depois DCOCTL
    DCOCTL  = CALDCO_1MHZ;
    BCSCTL2 = 0;
}