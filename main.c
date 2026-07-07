#include <msp430.h>

#define DIST_LIMIAR 10

void ini_uCon(void);
void ini_P1_P2(void);
void ini_Timer0_PWM_Trigger(void);
void ini_Timer1_Captura_Echo(void);
void ini_UART(void);
void atualizar_leds(void);
void uart_write_char(char c);
void uart_write_str(const char *str);
void uart_write_int(unsigned int val);

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
    ini_UART();
    atualizar_leds();

    do
    {
        if (distancia > 0)
        {
            atualizar_leds();

            // Transmite a distância medida via UART a cada ~500ms
            uart_write_str("Nivel - Distancia: ");
            uart_write_int(distancia);
            uart_write_str(" cm\r\n");
        }
        __delay_cycles(500000); // Aguarda 500ms para não lotar o buffer
    } while (1);
}

void atualizar_leds(void)
{
    unsigned char mask_p2 = 0, mask_p1 = 0;
    mask_p2 = BIT2 | BIT3 | BIT4 | BIT5 | BIT6 | BIT7;
    mask_p1 = BIT5 | BIT7; // LED 7 movido de P1.6 para P1.5 para liberar P1.6 para o Trigger

    P2OUT = (P2OUT & ~(BIT2 + BIT3 + BIT4 + BIT5 + BIT6 + BIT7)) | mask_p2;
    P1OUT = (P1OUT & ~(BIT5 + BIT7)) | mask_p1;
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
    // Configura direções dos pinos do Port 1:
    // P1.1 (RXD) como entrada da UART. O restante como saída.
    P1DIR = 0xFF & ~BIT1;
    P1OUT = 0;

    // Configura P1.6 como TA0.1 (PWM Trigger do HC-SR04)
    P1SEL |= BIT6;
    P1SEL2 &= ~BIT6;

    // Configura direções dos pinos do Port 2:
    // P2.1 (Echo) como entrada. O restante como saída.
    P2DIR = 0xFF & ~BIT1;
    P2OUT = 0;

    // Configura P2.1 como TA1.1 (Echo Capture)
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
void ini_UART(void)
{
    // Configura P1.1 (RXD) e P1.2 (TXD) para a função UART (USCI_A0)
    P1SEL |= BIT1 + BIT2;
    P1SEL2 |= BIT1 + BIT2;

    UCA0CTL1 |= UCSWRST;  // Coloca em Reset para configurar
    UCA0CTL1 |= UCSSEL_2; // Usa o clock SMCLK (1 MHz)
    UCA0BR0 = 104;        // 1 MHz / 9600 = 104 (Baud Rate 9600)
    UCA0BR1 = 0;
    UCA0MCTL = UCBRF_0 + UCBRS_1; // Configuração de modulação
    UCA0CTL1 &= ~UCSWRST;         // Libera o módulo UART para uso
}

void uart_write_char(char c)
{
    while (!(IFG2 & UCA0TXIFG))
        ;          // Aguarda buffer de TX ficar vazio
    UCA0TXBUF = c; // Transmite o caractere
}

void uart_write_str(const char *str)
{
    while (*str)
    {
        uart_write_char(*str++);
    }
}

void uart_write_int(unsigned int val)
{
    char buf[6];
    int i = 0;
    if (val == 0)
    {
        uart_write_char('0');
        return;
    }
    while (val > 0)
    {
        buf[i++] = (val % 10) + '0';
        val /= 10;
    }
    while (i > 0)
    {
        uart_write_char(buf[--i]);
    }
}