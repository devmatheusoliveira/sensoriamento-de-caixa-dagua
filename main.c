#include <msp430.h>

#define DIST_LIMIAR 10

void ini_uCon(void);
void ini_P1_P2(void);
void ini_Timer0_PWM_Trigger(void);
void ini_Timer1_Captura_Echo(void);

volatile unsigned int t_subida = 0;
volatile unsigned int largura = 0;
float distancia = 0;
volatile unsigned int indexAtual = 0;
float volumeAtual = 0;
float media = 0;
float raio = 28.5;         // mm
float alturaTotal = 200.0; // mm ou 20cm
float volumeTotal;         // em ml
const unsigned int qtdeAmostras = 32;
unsigned int amostrasDistancia[qtdeAmostras];

void main(void)
{
    volumeTotal = 3.14 * raio * raio * alturaTotal;
    ini_uCon();
    ini_P1_P2();
    ini_Timer0_PWM_Trigger();
    ini_Timer1_Captura_Echo();

    do
    {
    } while (1);
}

#pragma vector = TIMER1_A1_VECTOR
__interrupt void RTI_Captura_Echo(void)
{
    switch (TA1IV)
    {

    case TA1IV_TACCR1:

        if (TA1CCTL1 & COV)
        {
            TA1CCTL1 &= ~COV;
        }

        if (TA1CCTL1 & CCI)
        {
            t_subida = TA1CCR1;
        }
        else
        {
            if (indexAtual > qtdeAmostras)
            {
                indexAtual = 0;
            }
            largura = TA1CCR1 - t_subida;
            distancia = largura / 5.8; // para dar a distancia em mm
            amostrasDistancia[indexAtual] = distancia;
            indexAtual++;
        }
        break;

    default:
        break;
    }
}

void ini_Timer0_PWM_Trigger(void)
{
    TA0CTL = TASSEL1 + MC0;
    TA0CCTL0 = 0;
    TA0CCTL1 = OUTMOD2 + OUTMOD1 + OUTMOD0;
    TA0CCR0 = 59999;
    TA0CCR1 = 15;
}

void ini_Timer1_Captura_Echo(void)
{
    TA1CTL = TASSEL1 + MC1 + TACLR;
    TA1CCTL1 = CM1 + CM0 + SCS + CAP + CCIE;
}

void ini_P1_P2(void)
{
    P1DIR = 0xFF;
    P1OUT = 0;

    P1SEL |= BIT2;
    P1SEL2 &= ~BIT2;

    P2DIR = 0xFF;
    P2DIR &= ~BIT1;
    P2OUT = 0;

    P2SEL |= BIT1;
    P2SEL2 &= ~BIT1;
}

void ini_uCon(void)
{
    WDTCTL = WDTPW | WDTHOLD;

    DCOCTL = CALDCO_1MHZ;
    BCSCTL1 = CALBC1_1MHZ;
    BCSCTL2 = 0;

    __enable_interrupt();
}

float calcularMedia()
{
    int soma = 0;
    media = 0;

    for (int i = 0; i < qtdeAmostras; i++)
    {
        soma = +amostrasDistancia[i];
    }

    media = soma / qtdeAmostras;

    return media; // em mm
}

float calcularVolume()
{
    calcularMedia();
    float alturaAtual = 0;
    alturaAtual = alturaTotal - media;
    volumeAtual = (3.14 * raio * raio * alturaAtual) / 1000;

    return volumeAtual; // em ml
}