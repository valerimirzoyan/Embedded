#include <pic16f877a.h>

#define SBIT_TXEN      5
#define SBIT_SPEN      7
#define SBIT_CREN      4

void UART_Init(int baudRate)
{
    TRISC = 0x80;  // Set RX as input, TX as output
    TXSTA = (1 << SBIT_TXEN);  // Enable transmitter
    RCSTA = (1 << SBIT_SPEN) | (1 << SBIT_CREN);  // Enable serial port and continuous receive
    SPBRG = (20000000UL / (64UL * baudRate)) - 1;  // Set baud rate for 9600
}

void UART_TxChar(char ch)
{
    while (TXIF == 0);  // Wait for the transmit buffer to be empty
    TXREG = ch;  // Transmit the character
}

char UART_RxChar()
{
    while (RCIF == 0);  // Wait for the received character
    return RCREG;  // Return the received character
}

int main()
{
    char i;
    char a[] = "Hello World";  // String initialization

    UART_Init(9600);  // Initialize UART with 9600 baud rate
    
    while (1)
    {
        char ch = UART_RxChar();  // Receive a character
        
        if (ch == '#') {
            // Send the message "Hello World" when '#' is received
            for (i = 0; a[i] != 0; i++)
            {
                UART_TxChar(a[i]);  // Transmit each character
            }
        }
    }
}
