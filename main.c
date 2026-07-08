//==============================================================
// MSP430G2553 - HC-SR04 + 8 LEDs (bargraph) + Bomba/Ponte H + UART
// Media de 32 amostras + calculo de volume do reservatorio
//
// Mapa de pinos:
//   P1.1  - UCA0RXD (UART, jumpers do LaunchPad em HW UART)
//   P1.2  - UCA0TXD (UART)
//   P1.6  - Trigger HC-SR04 (saida TA0.1, pulso de 15 us a cada 30 ms)
//   P2.1  - Echo HC-SR04    (entrada de captura TA1.1)
//   P1.0  - Sinal digital p/ ponte H (bomba d'agua)
//   LEDs  - P2.0, P2.2, P2.3, P2.4, P2.5, P2.6, P2.7, P1.7
//
// Velocidade: periodo do trigger reduzido de 60 ms para 30 ms.
// Com QTDE_AMOSTRAS = 32, uma nova media sai a cada ~0,96 s
// (antes era ~1,92 s). A quantidade de amostras foi mantida.
//==============================================================
#include <msp430.h>

#define QTDE_AMOSTRAS 32

void ini_uCon(void);
void ini_P1_P2(void);
void ini_Timer0_PWM_Trigger(void);
void ini_Timer1_Captura_Echo(void);
void ini_UART(void);

void uart_write_char(char c);
void uart_write_str(const char *str);
void uart_write_float(float val);

float calcularMedia(void);
float calcularVolume(float media);
void atualizar_leds(unsigned int dist);
void controlarBomba(float volume);

volatile unsigned int t_subida = 0;
volatile unsigned int largura = 0;
volatile unsigned int indexAtual = 0;
volatile unsigned char amostraPronta = 0;
volatile unsigned int amostrasDistancia[QTDE_AMOSTRAS];

float volumeAtual = 0;
float mediaAtual = 0;
float raio = 28.5f;            // mm
float alturaTotal = 200.0f;    // mm
float volumeTotal;
float volumeMinimo = 115.0f;    // ml -> liga a bomba abaixo disso
float volumeMaximo = 460.0f;   // ml -> desliga a bomba acima disso

unsigned char bombaLigada = 0;

void main(void)
{
    volumeTotal = 3.14f * raio * raio * alturaTotal / 1000.0f;

    ini_uCon();
    ini_P1_P2();
    ini_Timer0_PWM_Trigger();
    ini_Timer1_Captura_Echo();
    ini_UART();

    __enable_interrupt();

    uart_write_str("Sistema iniciado\r\n");

    do
    {
        if (amostraPronta)
        {
            amostraPronta = 0;

            mediaAtual  = calcularMedia();
            volumeAtual = calcularVolume(mediaAtual);

            atualizar_leds((unsigned int)mediaAtual);
            controlarBomba(volumeAtual);

            uart_write_str("Distancia: ");
            uart_write_float(mediaAtual);
            uart_write_str(" cm\r\n");

            uart_write_str("Volume: ");
            uart_write_float(volumeAtual);
            uart_write_str(" ml\r\n");

            uart_write_str("Bomba: ");
            uart_write_str(bombaLigada ? "LIGADA\r\n" : "DESLIGADA\r\n");
        }
    } while (1);
}

//--------------------------------------------------------------
// Bargraph: quanto mais perto (tanque mais cheio), mais LEDs.
// dist < 5 cm -> 8 LEDs ... dist >= 40 cm -> 0 LEDs (1 LED a cada 5 cm)
//--------------------------------------------------------------
void atualizar_leds(unsigned int dist)
{
    static const unsigned char led_p2[8] = {BIT0, BIT2, BIT3, BIT4, BIT5, BIT6, BIT7, 0};
    static const unsigned char led_p1[8] = {0, 0, 0, 0, 0, 0, 0, BIT7};

    unsigned char n, i;
    unsigned char mask_p2 = 0, mask_p1 = 0;

    if (dist >= 20)
        n = 0;
    else
        n = 8 - (dist / 3);

    for (i = 0; i < n; i++)
    {
        mask_p2 |= led_p2[i];
        mask_p1 |= led_p1[i];
    }

    P2OUT = (P2OUT & ~(BIT0 + BIT2 + BIT3 + BIT4 + BIT5 + BIT6 + BIT7)) | mask_p2;
    P1OUT = (P1OUT & ~BIT7) | mask_p1;
}

//--------------------------------------------------------------
// Controle da bomba (ponte H em P1.0) com histerese:
// liga abaixo do volume minimo e so desliga ao atingir o maximo.
//--------------------------------------------------------------
void controlarBomba(float volume)
{
    if (volume < volumeMinimo)
    {
        P1OUT |= BIT0;
        bombaLigada = 1;
    }
    else if (volume >= volumeMaximo)
    {
        P1OUT &= ~BIT0;
        bombaLigada = 0;
    }
    // entre minimo e maximo: mantem o estado atual (histerese)
}

//--------------------------------------------------------------
// Captura do Echo em TA1.1 (P2.1): borda de subida guarda o
// instante inicial; borda de descida calcula a largura (us) e
// armazena no vetor de amostras.
//--------------------------------------------------------------
#pragma vector = TIMER1_A1_VECTOR
__interrupt void RTI_Captura_Echo(void)
{
    switch (TA1IV)
    {
    case TA1IV_TACCR1:

        if (TA1CCTL1 & COV)
        {
            TA1CCTL1 &= ~COV;           // descarta captura sobreposta
        }

        if (TA1CCTL1 & CCI)             // borda de subida
        {
            t_subida = TA1CCR1;
        }
        else                            // borda de descida
        {
            largura = TA1CCR1 - t_subida;   // aritmetica sem sinal trata o estouro
            amostrasDistancia[indexAtual] = largura;
            indexAtual++;
            if (indexAtual >= QTDE_AMOSTRAS)
            {
                indexAtual = 0;
                amostraPronta = 1;
            }
        }
        break;

    default:
        break;
    }
}

//--------------------------------------------------------------
// TA0: SMCLK (1 MHz), modo up, periodo 30 ms (era 60 ms).
// TA0.1 em modo 7 (reset/set): pulso de 15 us no P1.6 (Trigger).
// Obs.: 30 ms ainda comporta o echo maximo do HC-SR04 em uso
// normal; se o sensor ficar sem obstaculo (timeout ~38 ms),
// a captura de descida cai no ciclo seguinte e a amostra e
// naturalmente descartada pela media.
//--------------------------------------------------------------
void ini_Timer0_PWM_Trigger(void)
{
    TA0CTL   = TASSEL_2 | MC_1 | TACLR;
    TA0CCTL0 = 0;
    TA0CCTL1 = OUTMOD_7;
    TA0CCR0  = 29999;                   // 30 ms -> medicao 2x mais rapida
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
    UCA0MCTL  = UCBRF_0 + UCBRS_1;
    UCA0CTL1 &= ~UCSWRST;               // libera o modulo
}

void uart_write_char(char c)
{
    while (!(IFG2 & UCA0TXIFG));        // espera buffer livre
    UCA0TXBUF = c;
}

void uart_write_str(const char *str)
{
    while (*str)
        uart_write_char(*str++);
}

void uart_write_float(float val)
{
    char buf[12];
    int i = 0;
    long inteira;
    long decimal;
    long temp;

    if (val < 0)
    {
        uart_write_char('-');
        val = -val;
    }

    inteira = (long)val;
    decimal = (long)((val - (float)inteira) * 100.0f + 0.5f);

    if (decimal >= 100)
    {
        inteira += 1;
        decimal -= 100;
    }

    temp = inteira;

    if (temp == 0)
    {
        uart_write_char('0');
    }
    else
    {
        while (temp > 0)
        {
            buf[i++] = (char)((temp % 10) + '0');
            temp /= 10;
        }
        while (i > 0)
        {
            uart_write_char(buf[--i]);
        }
    }

    uart_write_char(',');
    uart_write_char((char)((decimal / 10) + '0'));
    uart_write_char((char)((decimal % 10) + '0'));
}

float calcularMedia(void)
{
    unsigned long soma = 0;
    unsigned int i;
    float media;

    __disable_interrupt();
    for (i = 0; i < QTDE_AMOSTRAS; i++)
    {
        soma += amostrasDistancia[i];
    }
    __enable_interrupt();

    media = ((float)soma / QTDE_AMOSTRAS) / 58.0f;   // us -> cm

    return media;
}

float calcularVolume(float media)
{
    float alturaAtual;
    float raioCm;
    float volume;

    alturaAtual = (alturaTotal / 10.0f) - media;     // cm
    if (alturaAtual < 0)
    {
        alturaAtual = 0;
    }

    raioCm = raio / 10.0f;                           // cm
    volume = 3.14f * raioCm * raioCm * alturaAtual;  // ml

    return volume;
}

void ini_uCon(void)
{
    WDTCTL = WDTPW | WDTHOLD;

    BCSCTL1 = CALBC1_1MHZ;              // primeiro BCSCTL1, depois DCOCTL
    DCOCTL  = CALDCO_1MHZ;
    BCSCTL2 = 0;
}