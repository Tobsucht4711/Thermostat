/*
 * Thermostat.c
 *
 * Created: 03.10.2019 17:05:17
 * Author : u016346
 */ 

#define F_CPU 16000000UL
#define FIRST_TX_PDO 16
#define SECOND_TX_PDO 32
#define SDO_RESPONSE 48
#define HEARTBEAT 64
#define FIRST_RX_PDO 80
#define SDO_REQUEST 96
#define NODE_ID 4

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

volatile uint16_t ADC_Value;
volatile uint16_t Counter;
volatile uint8_t TimerCounter;
volatile uint8_t FirstCycle = 1;

void can_init(uint8_t NodeID)
{
	uint8_t i,j;
	uint16_t id;

	CANGCON = (1<<SWRES); // off and reset

	for (i=0; i<6; i++)
	{
		CANPAGE = (i<<4); // select MOb
		CANCDMOB = 0x00;	// disable MOb
		CANSTMOB = 0x00;	// clear status
		CANIDT1 = 0x00;		// clear ID
		CANIDT2 = 0x00;
		CANIDT3 = 0x00;
		CANIDT4 = 0x00;
		CANIDM1 = 0x00;		// clear mask
		CANIDM2 = 0x00;
		CANIDM3 = 0x00;
		CANIDM4 = 0x00;
		for (j=0; j<8; j++)
		{
			CANMSG = 0x00; // clear data
		}
	}

	// setup für 125 kBit/s mit 16 MHz Quarz
	CANBT1 = 0x0E;
	CANBT2 = 0x0C;
	CANBT3 = 0x37;

	CANGCON = (1<<ENASTB); // enable CAN

	id = 0x180 + NodeID;   // Sende-ID
	CANPAGE = FIRST_TX_PDO; // select MOB0
	CANIDT1 = (id >> 3); // Put bits 3-11 of message ID into CANIDT1
	CANIDT2 = (id << 5); // Put bits 0-2 of message ID into CANIDT2
	CANIDM1 = 0xff;
	CANIDM2 = 0xe0;
	CANIDM4 = (1<<IDEMSK);
	CANCDMOB = (1<<DLC0); // 1 Byte
	CANSTMOB |= (1<<TXOK); // set flag

	id = 0x280 + NodeID;   // Sende-ID
	CANPAGE = SECOND_TX_PDO; // select MOB0
	CANIDT1 = (id >> 3); // Put bits 3-11 of message ID into CANIDT1
	CANIDT2 = (id << 5); // Put bits 0-2 of message ID into CANIDT2
	CANIDM1 = 0xff;
	CANIDM2 = 0xe0;
	CANIDM4 = (1<<IDEMSK);
	CANCDMOB = (1<<DLC2) | (1<<DLC1); // 6 Byte
	CANSTMOB |= (1<<TXOK); // set flag
	
	id = 0x580 + NodeID;   // Sende-ID
	CANPAGE = SDO_RESPONSE; // select MOB0
	CANIDT1 = (id >> 3); // Put bits 3-11 of message ID into CANIDT1
	CANIDT2 = (id << 5); // Put bits 0-2 of message ID into CANIDT2
	CANIDM1 = 0xff;
	CANIDM2 = 0xe0;
	CANIDM4 = (1<<IDEMSK);
	CANCDMOB = (1<<DLC3); // 8 Byte
	CANSTMOB |= (1<<TXOK); // set flag

	id = 0x700 + NodeID;   // Sende-ID
	CANPAGE = HEARTBEAT; // select MOB0
	CANIDT1 = (id >> 3); // Put bits 3-11 of message ID into CANIDT1
	CANIDT2 = (id << 5); // Put bits 0-2 of message ID into CANIDT2
	CANIDM1 = 0xff;
	CANIDM2 = 0xe0;
	CANIDM4 = (1<<IDEMSK);
	CANCDMOB = (1<<DLC0); // 1 Byte
	CANSTMOB |= (1<<TXOK); // set flag
	
	id = 0x200 + NodeID;   // Empfangs-ID
	CANPAGE = FIRST_RX_PDO; // select MOB0
	CANIDT1 = (id >> 3); // Put bits 3-11 of message ID into CANIDT1
	CANIDT2 = (id << 5); // Put bits 0-2 of message ID into CANIDT2
	CANIDM1 = 0xff;
	CANIDM2 = 0xe0;
	CANIDM4 = (1<<IDEMSK);
	CANCDMOB = (1<<CONMOB1) | (1<<DLC0); // 1 Byte
	CANSTMOB = 0x00;
	
	id = 0x600 + NodeID;   // Empfangs-ID
	CANPAGE = SDO_REQUEST; // select MOB0
	CANIDT1 = (id >> 3); // Put bits 3-11 of message ID into CANIDT1
	CANIDT2 = (id << 5); // Put bits 0-2 of message ID into CANIDT2
	CANIDM1 = 0xff;
	CANIDM2 = 0xe0;
	CANIDM4 = (1<<IDEMSK);
	CANCDMOB = (1<<CONMOB1) | (1<<DLC3); // 8 Byte
	CANSTMOB = 0x00;

	CANGIE = (1<<ENIT) | (1<<ENRX);
	CANEN1 = 0x00;
	CANEN2 = (1<<ENMOB5) | (1<<ENMOB4) | (1<<ENMOB3) | (1<<ENMOB2) | (1<<ENMOB1) | (1<<ENMOB0); // enable MOB0
	CANIE1 = 0x00;
	CANIE2 = (1<<IEMOB5) | (1<<IEMOB4);
	CANSIT1 = 0x00;
	CANSIT2 = 0x00;
}

void ADC_Init(void){
	ADMUX = (1<<REFS0) | (1<<MUX3) | (1<<MUX1);
	ADCSRA = (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
	ADCSRB = (1<<AREFEN);
	ADCSRA |= (1<<ADEN);
	ADCSRA |= (1<<ADSC);
	while (ADCSRA & (1<<ADSC) ) {}
	(void) ADCW;
}

ISR (CAN_INT_vect)
{
	uint8_t tmp_canpage;

	tmp_canpage = CANPAGE; // save canpage

	if(CANSIT2 & SIT4) // MOB4
	{
		// first receive PDO
		CANPAGE = FIRST_RX_PDO; // select MOB1
		CANSTMOB &= ~(1<<RXOK); // clear interrupt flag
		CANCDMOB = (1<<CONMOB1) | (1<<DLC0); // 1 Byte
		PORTC = (PORTC & ~((1<<PC4) | (1<<PC5))) | ((CANMSG << 4) & ((1<<PC4) | (1<<PC5)));
	}
	if(CANSIT2 & SIT5) // MOB5
	{
		uint8_t IndexLow, IndexHigh, IndexSub;
		// SDO request
		CANPAGE = SDO_REQUEST; // select MOB1
		CANSTMOB &= ~(1<<RXOK); // clear interrupt flag
		CANCDMOB = (1<<CONMOB1) | (1<<DLC3); // 8 Byte
		(void)CANMSG;
		IndexLow = CANMSG;
		IndexHigh = CANMSG;
		IndexSub = CANMSG;
		(void)CANMSG;
		(void)CANMSG;
		(void)CANMSG;
		(void)CANMSG;
		CANPAGE = SDO_RESPONSE;
		if(CANSTMOB & (1<<TXOK)) // fertig mit Senden?
		{
			CANSTMOB &= ~(1<<TXOK); // reset flag
			CANCDMOB = (1<<DLC3); // 8 Byte
			CANMSG = 0x80;
			CANMSG = IndexLow;
			CANMSG = IndexHigh;
			CANMSG = IndexSub;
			CANMSG = 0x00;
			CANMSG = 0x00;
			CANMSG = 0x02;
			CANMSG = 0x06;
			CANCDMOB |= (1<<CONMOB0); // Transfer einleiten
		}
	}
	CANPAGE = tmp_canpage; // restore canpage
}

//TIMER0_COMPA_vect
ISR (TIMER0_COMPA_vect)
{
	if ((++Counter % 50) == 1)
	{
		CANPAGE = HEARTBEAT;
		if(CANSTMOB & (1<<TXOK)) // fertig mit Senden?
		{
			CANSTMOB &= ~(1<<TXOK); // reset flag
			CANCDMOB = (1<<DLC0); // 1 Byte
			// Heartbeat senden
			if (FirstCycle)
			{
				FirstCycle = 0;
				// Boot Up senden (00)
				CANMSG = 0x00;
			}
			else
			{
				// Heartbeat senden (05)
				CANMSG = 0x05;
			}
			CANCDMOB |= (1<<CONMOB0); // Transfer einleiten
		}
	}
	if (Counter == 100)
	{
		// 2. Tx PDO senden
		CANPAGE = SECOND_TX_PDO;
		if(CANSTMOB & (1<<TXOK)) // fertig mit Senden?
		{
			CANSTMOB &= ~(1<<TXOK); // reset flag
			CANCDMOB = (1<<DLC2) | (1<<DLC1); // 6 Byte
			CANMSG = ADC_Value & 0xFF;
			CANMSG = (ADC_Value >> 8) & 0xFF;
			CANMSG = 0x12;
			CANMSG = 0x34;
			CANMSG = 0x56;
			CANMSG = 0x78;
			CANCDMOB |= (1<<CONMOB0); // Transfer einleiten
		}
		Counter = 0;
	}
}

void Timer_Init(void)
{
	TCCR0A = (1<<WGM01); // Mode 2 CTC
	TCCR0B |= (1<<CS02) | (1<<CS01) | (1<<CS00);  // Vorteiler 1024
	OCR0A = 155;	// 1 / (16000000 / 1024 / 156) = 9,984 ms
	TIMSK0 |= (1<<OCIE0A); // | (1<<TOIE0);
}

int main(void)
{
    volatile uint8_t Cmp_Taster = 0;
    DDRC = (1<<DDC2) | (1<<DDC4) | (1<<DDC5);
    PORTB = (1<<PB2);
    PORTC = (1<<PC3) | (1<<PC5);
    can_init(NODE_ID);
    _delay_ms(500);
    PORTC |= (1<<PC4);
    ADC_Init();
    _delay_ms(500);
    PORTC &= ~(1<<PC5);
    Timer_Init();
    _delay_ms(500);
    PORTC &= ~(1<<PC4);
    sei();
    while(1)
    {
	    ADCSRA |= (1<<ADSC);
	    while (ADCSRA & (1<<ADSC) ) {}
	    ADC_Value = ADCW;
	    _delay_ms(50);
		TimerCounter = TCNT0;
	    if((PINB & (1<<PB2)) != Cmp_Taster )
	    {
		    // 1. Tx PDO senden
		    CANPAGE = FIRST_TX_PDO;
		    if(CANSTMOB & (1<<TXOK)) // fertig mit Senden?
		    {
			    CANSTMOB &= ~(1<<TXOK); // reset flag
			    CANCDMOB = (1<<DLC0); // 1 Byte
			    CANMSG = ((PINB & (1<<PB2)) > 0) ^ 1;
			    CANCDMOB |= (1<<CONMOB0); // Transfer einleiten
		    }
		    Cmp_Taster = (PINB & (1<<PB2));
	    }
    }
}

