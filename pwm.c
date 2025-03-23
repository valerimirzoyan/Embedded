#include <pic.h>
#include <math.h>

#define Speed 100
#define _XTAL_FREQ 20000000 ///

#define DownState 1
#define UpState 2
#define DownUpState 3

#define LED0 RD1

void Timer1_Init(void);

unsigned char 	DataOut;
unsigned int 	Counter, DebounceCnt, DataS, activeCount, current;
unsigned char 	inFlag;
unsigned char 	outFlag,outState;
unsigned char rxbyte[2];
unsigned char uart_event;
unsigned char tx[] = "BAREV DZEZ\r\n";
int i,j,cnt,k;
unsigned char pwmCounter = 0;
unsigned char ledBrightness = 0;
unsigned char ledPWM=255;
float temp;
unsigned int adcData;
unsigned char adcflag;
unsigned int adcCount;

void SetPWMDutyCycle(unsigned int DutyCycle){
	DutyCycle = 4*DutyCycle;
	CCPR1L = DutyCycle>>2;
	CCP1CON &= 0xCF;
	CCP1CON |= (0x30&(DutyCycle<<4));
}

void interrupt ISR(void){
	if(TMR1IF){

		TMR1IF = 0;
		Counter++;
	j++;
	if(j >= 10){
		j = 0;
		ledPWM--;
			
	}
	adcCount++;
	if(adcCount >= 100){
		adcCount = 0;
		adcflag = 1;
	}
	if(ledPWM==0) ledPWM = 255;
	SetPWMDutyCycle(ledPWM);
	
	

	//if (ledBrightness > pwmCounter) LED0 = 1; else LED0 = 0;
	pwmCounter++;
	if (pwmCounter > 19) pwmCounter = 0;

		TMR1H = 0xFD;
		TMR1L = 0x8E;
	}

	if(RCIF){
		RCIF = 0;
		rxbyte[i] = RCREG;
		//if(rxbyte[i-1] == '#'){
			if(rxbyte[i] == '$'){
				uart_event = 1;
		}
		i++;
		if(i > 1) i = 0;
	}

	CLRWDT();
}

void SPI_Write(unsigned char incoming)
{
    SSPBUF = incoming; //Write the user given data into buffer
	while ( !SSPSTATbits.BF );
}

unsigned char SPI_Read() //Read the received data
{
    while ( !SSPSTATbits.BF ); // Hold till BF bit is set, to make sure the complete data is read
    return(SSPBUF); // return the read data
}

void I2C_Start(){
	while((SSPSTAT & 0x04) || (SSPCON2 & 0x1F));
	SEN = 1;
}
void I2C_Stop(){
	while((SSPSTAT & 0x04) || (SSPCON2 & 0x1F));
	PEN = 1;
}
void I2C_Repeated_Start(){
	while((SSPSTAT & 0x04) || (SSPCON2 & 0x1F));
	RSEN = 1;
}
void I2C_Write(unsigned char i2c_data){
	while((SSPSTAT & 0x04) || (SSPCON2 & 0x1F));
	 SSPBUF = i2c_data;	
}

unsigned char I2C_Read(unsigned char ack){
	unsigned char _data;
	while((SSPSTAT & 0x04) || (SSPCON2 & 0x1F));
	RCEN = 1;
	while((SSPSTAT & 0x04) || (SSPCON2 & 0x1F));
	_data = SSPBUF;
	while((SSPSTAT & 0x04) || (SSPCON2 & 0x1F));
	ACKDT = (ack)?0:1;
	ACKEN = 1;
	return _data;
}


unsigned int ADC_Read(unsigned char channel)
{
	ADCON0 = 0b10000001; //ADC ON and Fosc/32 is selected
  	ADCON1 = 0b10000010; // Internal reference voltage is selected
  	TRISA = 0x3F;
  //ADCON0 &= 0x11000101; //Clearing the Channel Selection Bits
	ADCON0bits.CHS0 = 0;
	ADCON0bits.CHS1 = 1;
	ADCON0bits.CHS2 = 0;	
  //ADCON0 |= channel<<3; //Setting the required Bits
  //__delay_ms(1); //Acquisition time to charge hold capacitor
  GO_nDONE = 1; //Initializes A/D Conversion
 /* while(GO_nDONE){ //Wait for A/D Conversion to complete
__delay_ms(1);
}*/
  return ((ADRESH<<8)+ADRESL); //Returns Result
}

int main(){
	PEIE = 1;
	GIE = 1;
	TRISB = 0b00000000;
	TRISE = 3;
	TRISC = 0b00000000;
	TRISD = 0b00000000;
	
	PORTB = 0xFF;
	
	DataOut = 0;
	Counter = 0;
	inFlag = 0;
	outState = DownState;
	DataS = 1;
	activeCount = 0;
    current = 1;
	////////////////////ADC init///////////////////////
	ADCON0 = 0b10000001; //ADC ON and Fosc/32 is selected
  	ADCON1 = 0b10000010; // Internal reference voltage is selected
  	TRISA = 0x3F;
	/*************Timer 1 init*************/
	/*T1CON = 0x31;
	TMR1IF = 0;
	TMR1IE = 1;
	PEIE = 1;
	GIE = 1;
	TMR1H = 0xF3;
	TMR1L = 0xCB;*/
	Timer1_Init();
	_nop();

/**********************************************/
	////////UART init//////////
	TRISC6 = 0; //TX pin
	TRISC7 = 1; //RX pin
	
	SPBRG = 129;
	BRGH = 1; //high speed
	SYNC = 0; //asynchronous
	SPEN = 1; //enable serial pin
	RX9 = 0; //8 bit reception
	SREN = 0; //no effect
	CREN = 1; //enable reception 
	TX9 = 0; //8 bit reception
	TXEN = 0; //reset transmitter
	TXEN = 1; //enable transmitter
	RCIE = 1; //enable RX interrupt

////*****I2C init************//////////////
	SSPCON = 0x28;
	SSPSTAT = 0;
	SSPADD = (20000000 / (4 * (100000)))-1;
	TRISC3 = 1;  
	TRISC4 = 1;

///*****SPI init**********/////////////////
	/*TRISC3 = 0; //Clock pin
	TRISC5 = 0; //Data pin
	SSPSTAT = 0b00000000; 
	SSPCON = 0b00100010; 
	SSPIF = 0;*/
	//SSPIE = 1;
//////////******PWM init*******///////////
	CCP1CON = 0x0C;
	PR2 = 0xFF;
	T2CON = 0x01;
	T2CON |= 0x04;
//SetPWMDutyCycle(200);

////////////////////////////////
	while(1){
		if(adcflag){
			adcData = ADC_Read(1);
			PORTD = adcData&0xFF; 
			adcflag = 0;
			I2C_Start();
			I2C_Write(adcData);
			I2C_Write((adcData>>8));
			I2C_Stop();
			
		}
		/*for(k=0;k<99;k++) {
	     temp=100*(sin(2*3.14159*k/100)+1);
	     SetPWMDutyCycle((unsigned char)temp);
	 	}*/
		for (cnt = 0; cnt < 20; cnt++)
		{
			ledBrightness = cnt;
		
			// Pause for a bit
		//	for (int pause = 0; pause < 30; pause++) delay(100);
		}
		
		if(uart_event){
			uart_event = 0;
			
			for(int cc=0; cc<13; cc++){
				TXREG = tx[cc];
				while(!TRMT);
			}
		/*	I2C_Start();
			I2C_Write(0x09);
			I2C_Repeated_Start();
			I2C_Write(0x22);*/
			//I2C_Write(0x33);
			I2C_Stop();
		/*	SPI_Write(0x12);
			SPI_Write(0x34);
			SPI_Write(0x56);
			SPI_Write(0x78);*/
		}
		/*if(SPI_DataReady()){//slave mode
			data =  SPI_Read();
		}*/

		if(RE0 == 0){
			DebounceCnt++;
			if(DebounceCnt >= 200){
				DebounceCnt  = 0;
				inFlag = 1;
			}
		}else{
			inFlag = 0;
		}

		if(RE0 == 1)
		{
			DataS = 1;	
		}
		if(inFlag){
			//Counter++;
			if(Counter >= Speed){
				Counter = 0;
				DataOut = 1;
				PORTB = DataS;
				if(DataS == 128) outFlag = 0;
				if(DataS == 1) outFlag = 1;
			}
			if(DataOut){
				DataOut = 0;
				/*if(outFlag) DataS <<= 1;
				if(outFlag == 0) DataS >>= 1;*/
				switch(outState)
				{
					case DownState:
						DataS = (1 << current - 1) | ~(255 >> activeCount);//00000000 |
						if(current == 8 - activeCount)
						{
							activeCount++;
							current = 0;
						}
						current++;
						if(activeCount >= 8)
						{
							outState = UpState;
							current = 0;
						}
						break;
					case UpState:
						DataS = (0b10000000 >> (activeCount	+ current)) | ~(255 >> activeCount);
						if(current == 8 - activeCount)
						{
							activeCount--;
							current = 0;
						}
						current++;
						if(activeCount >= 8){
							current = 1;
							activeCount = 0;
							outState = DownState;
						}
						break;
					//case DownUpState:
					//	DataS >>= 1;
					//	break;
				}
			
			}
		}
	}
}

void Timer1_Init(void){
	T1CON = 0x31;
	TMR1IE = 1;
	TMR1IF = 0;
	TMR1H = 0xFD;
	TMR1L = 0x8E;
	
}
