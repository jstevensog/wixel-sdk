/* battery.c - Brief test of battery monitoring code.
*/
#include <wixel.h>
#include <usb.h>
#include <usb_com.h>
#include <stdio.h>

#define BATTERY_MAXIMUM		(521)
#define BATTERY_MINIMUM		(347)

static volatile BIT usb_connected;


void putchar(char c)
{
	//if (usb_connected)
		usbComTxSendByte(c);
}

// function to convert a voltage value to a battery percentage
uint8 batteryPercent(uint16 val){
	float pct=val;
	if(val < BATTERY_MINIMUM)
		return 0;
	if(val > BATTERY_MAXIMUM)
		return 100;
	pct = ((pct - BATTERY_MINIMUM)/(BATTERY_MAXIMUM-BATTERY_MINIMUM))*100;
	printf("%i, %i, %i, %i\r\n", val, BATTERY_MINIMUM, BATTERY_MAXIMUM, (uint8)pct);
//	delayMs(2500);
	return (uint8)pct;
}

void LineStateChangeCallback(uint8 state)
{
	//LED_YELLOW(state & ACM_CONTROL_LINE_DTR);
	usb_connected = state & ACM_CONTROL_LINE_DTR;
}


/** Functions *****************************************************************/
/* the functions that puts the system to sleep (PM2) and configures sleep timer to
wake it again in 250 seconds.*/
void makeAllOutputs(BIT value)
{
	//we only make the P1_ports low, and not P1_2 or P1_3
    int i = 10;
    for (;i < 17; i++)
	{
		//we don't set P1_2 low, it stays high.
/*		if(i==12)
			setDigitalOutput(i, HIGH);
		else if(i==13)
			setDigitalInput(i, PULLED);
		else
*/			setDigitalOutput(i, value);
    }
}

void main()
{   
	uint32 last_ms;
	systemInit();
	//configure the P1_2 and P1_3 IO pins
	makeAllOutputs(LOW);
	//initialise Anlogue Input 0
	P0INP = 0x1;
	//initialise the USB port
	usbInit();

	usbComRequestLineStateChangeNotification(LineStateChangeCallback);
	
	last_ms = getMs();
	while (1)
	{
		boardService();
		usbComService();
		if((getMs()-last_ms) >=5000){
			LED_YELLOW_TOGGLE();
			printf("batteryPercent: %i\r\n", batteryPercent(adcRead(0 | ADC_REFERENCE_INTERNAL)));
			last_ms=getMs();
		}
	}
}
