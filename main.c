#include <msp430.h>

#define DIST_LIMIAR 10
#define ALTURA_TANQUE 100

// Pinos de controle do 74HC595
#define DS_PIN    BIT7   // P1.7 - Dado Serial (DS)
#define SHCP_PIN  BIT4   // P1.4 - Clock de Deslocamento (SHCP)
#define STCP_PIN  BIT5   // P1.5 - Clock de Armazenamento / Latch (STCP)

void ini_uCon(void);
void ini_P1_P2(void);
void ini_Timer0_PWM_Trigger(void);
void ini_Timer1_Captura_Echo(void);
void enviar_595(unsigned int dados);
void atualizar_leds(unsigned int num_leds);

volatile unsigned int t_subida = 0;
volatile unsigned int largura = 0;
volatile unsigned int distancia = 0;



void main(void)
{
    ini_uCon();
    ini_P1_P2();
    ini_Timer0_PWM_Trigger();
    ini_Timer1_Captura_Echo();
    atualizar_leds(0);

    do {
        if (distancia > 0) {
            unsigned int leds = 0;
            if (distancia <= DIST_LIMIAR) {
                leds = 10;
            } else if (distancia >= ALTURA_TANQUE) {
                leds = 0;
            } else {
                // Mapeia linearmente a distância (DIST_LIMIAR a ALTURA_TANQUE) em 10 a 0 LEDs
                leds = 10 - ((distancia - DIST_LIMIAR) * 10 / (ALTURA_TANQUE - DIST_LIMIAR));
            }
            atualizar_leds(leds);
        }
        __delay_cycles(100000); // Atualiza a barra de LEDs a cada ~100ms
    } while (1);
}

void enviar_595(unsigned int dados)
{
    int i;
    for (i = 15; i >= 0; i--) {
        if (dados & (1 << i)) {
            P1OUT |= DS_PIN;
        } else {
            P1OUT &= ~DS_PIN;
        }

        // Pulso de clock no SHCP
        P1OUT |= SHCP_PIN;
        __delay_cycles(2);
        P1OUT &= ~SHCP_PIN;
    }

    // Pulso no latch clock STCP
    P1OUT |= STCP_PIN;
    __delay_cycles(2);
    P1OUT &= ~STCP_PIN;
}

void atualizar_leds(unsigned int num_leds)
{
    unsigned int mascara = 0;
    unsigned int i;

    // Liga num_leds consecutivos
    for (i = 0; i < num_leds; i++) {
        mascara |= (1 << i);
    }

    enviar_595(mascara);
}

#pragma vector = TIMER1_A1_VECTOR
__interrupt void RTI_Captura_Echo(void)
{
    switch (TA1IV) {

    case TA1IV_TACCR1:

        if (TA1CCTL1 & COV) {
            TA1CCTL1 &= ~COV;
        }

        if (TA1CCTL1 & CCI) {
            t_subida = TA1CCR1;
        } else {
            largura = TA1CCR1 - t_subida;
            distancia = largura / 58;
            if (distancia < DIST_LIMIAR) {
                P1OUT |= BIT0;
            } else {
                P1OUT &= ~BIT0;
            }
        }
        break;

    default:
        break;
    }
}

void ini_Timer0_PWM_Trigger(void)
{
    TA0CTL   = TASSEL1 + MC0;
    TA0CCTL0 = 0;
    TA0CCTL1 = OUTMOD2 + OUTMOD1 + OUTMOD0;
    TA0CCR0  = 59999;
    TA0CCR1  = 15;
}

void ini_Timer1_Captura_Echo(void)
{
    TA1CTL   = TASSEL1 + MC1 + TACLR;
    TA1CCTL1 = CM1 + CM0 + SCS + CAP + CCIE;
}

void ini_P1_P2(void)
{
    // P1.1 (RXD) como entrada se UART for usada. O restante como saída.
    P1DIR = 0xFF & ~BIT1;
    P1OUT = 0;

    // Configura P1.2 como TA0.1 (PWM Trigger)
    P1SEL  |= BIT2;
    P1SEL2 &= ~BIT2;

    // P2.1 como entrada (Echo do sensor). O restante como saída.
    P2DIR  = 0xFF & ~BIT1;
    P2OUT  = 0;

    // Configura P2.1 como TA1.1 (Echo Capture)
    P2SEL  |= BIT1;
    P2SEL2 &= ~BIT1;
}

void ini_uCon(void)
{
    WDTCTL = WDTPW | WDTHOLD;

    DCOCTL  = CALDCO_1MHZ;
    BCSCTL1 = CALBC1_1MHZ;
    BCSCTL2 = 0;

    __enable_interrupt();
}