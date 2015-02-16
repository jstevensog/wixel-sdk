#include <wixel.h>
#include <usb.h>
#include <usb_com.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>


#define SLEEP_MODE_USING (0x01)

static volatile BIT is_sleeping;
static volatile BIT do_sleep;
static volatile BIT sent_beacon;
static volatile BIT do_close_usb = 0;
static volatile BIT usb_connected;
static uint8 sleep_mode = 0;
static volatile uint32 pkt_time;
static uint8 save_IEN0;
static uint8 save_IEN1;
static uint8 save_IEN2;


// This is called by printf and printPacket.
void putchar(char c)
{
	usbComTxSendByte(c);
}

void LineStateChangeCallback(uint8 state)
{
	//LED_YELLOW(state & ACM_CONTROL_LINE_DTR);
	usb_connected = state & ACM_CONTROL_LINE_DTR;
}

void makeAllOutputs(BIT value)
{
	//we only make the P2_ports low
    int i;
    for (i=10;i < 17; i++)
	{
			setDigitalOutput(i, value);
    }
}

// ISR for catching Sleep Timer interrupts
ISR (ST, 0) 
{
	IRCON &= ~0x80; 	// clear IRCON.STIF
	//SLEEP &= ~SLEEP_MODE_USING; 	// clear SLEEP.MODE
	IEN0 &= ~0x20; 		// clear IEN0.STIE
	IEN1 = save_IEN1;
	IEN2 = save_IEN2;
	SLEEP &= ~sleep_mode; 	// clear SLEEP.MODE
	WORIRQ &= ~0x11; 	// clear Sleep timer EVENT0_MASK and EVENT0_FLAG
	WORCTRL &= ~0x03; 	// Set timer resolution back to 1 period.
//	U1UCR |= 0x80;	// flush UART1.
	

	if(do_close_usb)
	{
		// wake up USB again
		usbPoll();
	}
	// we not sleeping no more
	is_sleeping = 0; 
}

void goToSleep (uint16 seconds) {
    unsigned char temp;
	uint16 sleep_time;
    // The wixel docs note that any high output pins consume ~30uA
    makeAllOutputs(LOW);
	/*P1_0 = 0;
	P1DIR |= 1;
	// make sure HM-1x is depowered.
	while(P1_0 == 1) { 
		doServices(0); 
	}
	*/
	LED_YELLOW(1);

    //IEN0 |= 0x20; // Enable global ST interrupt [IEN0.STIE]
	//save_IEN0 = IEN0;
	IEN0 = 0xA0; // Enable global ST interrupt [IEN0.EA and IEN0.STIE]
	save_IEN1 = IEN1;
	IEN1 &= 0xB0;
	save_IEN2 = IEN2;
	IEN2 &= 0xB0;
    WORIRQ |= 0x10; // enable sleep timer interrupt [EVENT0_MASK]

	/* the sleep mode i've chosen is PM2. According to the CC251132 datasheet,
	typical power consumption from the SoC should be around 0.5uA */
	/*The SLEEP.MODE will be cleared to 00 by HW when power
	mode is entered, thus interrupts are enabled during power modes.
	All interrupts not to be used to wake up from power modes must
	be disabled before setting SLEEP.MODE!=00.*/
    
	sleep_time = (uint16)(((seconds*1000) - (getMs() - pkt_time))/1000);
	//sleep_time = seconds;
	printf("seconds: %u, sleep_time: %u, now-pkt_time: %lu\r\n", seconds, sleep_time, (getMs()-pkt_time));
	while(usbComTxAvailable() < 128) {
		usbComService();
	}
	if(sleep_time <= 0 || sleep_time > seconds) {
		do_sleep = 0;
		pkt_time = 0;
		return;
	}

	if(!usb_connected)
	{
		disableUsbPullup();
		usbDeviceState = USB_STATE_DETACHED;
		// disable the USB module
		SLEEP &= ~(1<<7); // Disable the USB module (SLEEP.USB_EN = 0).
		// sleep power mode 2 is incompatible with USB - as USB registers lose state in this mode.
		sleep_mode = SLEEP_MODE_USING;
		//SLEEP |= sleep_mode; // SLEEP.MODE = PM2
	} else {
		// As the USB is connected, we are going into PM1.
		sleep_mode = 0x01;
	}
	

	//ensure the HS XOSC is stable before we enter sleep mode.
	//while(!(SLEEP && 0x80)) {}
	// make sure we are configured to use the correct clock (32.768kHz crystal)
	//CLKCON &= 0x3F;
    // Reset timer, update EVENT0, and enter PM2
    // WORCTRL[2] = Reset Timer
    // WORCTRL[1:0] = Sleep Timer resolution
    // 00 = 1 period
    // 01 = 2^5 periods
    // 10 = 2^10 periods
    // 11 = 2^15 periods

    // t(event0) = (1/32768)*(WOREVT1 << 8 + WOREVT0) * timer res
    // e.g. WOREVT1=0,WOREVT0=1,res=2^15 ~= 0.9766 second

    WORCTRL |= 0x03; // 2^15 periods
    WORCTRL |= 0x04; // Reset
    // Wait for 2x+ve edge on 32kHz clock
    temp = WORTIME0;
    while (temp == WORTIME0) {};
    temp = WORTIME0;
    while (temp == WORTIME0) {};

/*    // Wait for 2x+ve edge on 32kHz clock
    temp = WORTIME0;
    while (temp == WORTIME0) {};
//    temp = WORTIME0;
//    while (temp == WORTIME0) {};
*/
    WOREVT1 = (sleep_time >> 8);
    WOREVT0 = (sleep_time & 0xff); //300=293 s

	SLEEP |= sleep_mode;	// set the sleep mode.
	__asm nop __endasm;		// We have extra NOPs to be safe.
	__asm nop __endasm;
	__asm nop __endasm;

	if((SLEEP && sleep_mode) != 0 ) PCON |= 0x01; // PCON.IDLE = 1;
	// tell everything we are sleeping
	is_sleeping = 1;
	LED_YELLOW(0);
}

// function to wait a specified number of milliseconds, whilst processing services.
void waitDoingServices (uint32 wait_time) {
	uint32 start_wait;
	start_wait = getMs();
	while ((getMs() - start_wait) < wait_time) {
		boardService();
		usbComService();
	}
}

void main()
{   
	systemInit();
	//initialise the USB port
	usbInit();
	//initialise the dma channel for working with flash.
	//dma_Init();
	// implement the USB line State Changed callback function.  
	usbComRequestLineStateChangeNotification(LineStateChangeCallback);
	// Open the UART and set it up for comms to HM-10
	//turn on HM-1x using P1_0
	setDigitalOutput(10,HIGH);
	//wait 4 seconds, just in case it needs to settle.
	waitDoingServices(4000);
	sent_beacon = 0;
	LED_GREEN(1);

	// MAIN LOOP
	while (1)
	{
		LED_RED_TOGGLE();
		waitDoingServices(5000);
		pkt_time = getMs();
		printf("pkt_time: %lu\r\n", pkt_time);
		waitDoingServices(1000);
		do_sleep = 1;
		if (do_sleep && !is_sleeping)
		{
			// not sure what this is about yet, but I believe it is saving state.
		    uint8 savedPICTL = PICTL;
			BIT savedP0IE = P0IE;
			//do_sleep = 0;

			RFST = 4;   //SIDLE
			// wait 500 ms, processing services.
			// sleep for around 300s
			goToSleep(10);   //
			PICTL = savedPICTL;
			P0IE = savedP0IE;
			// Enable suspend detection and disable any other weird features.
			USBPOW = 1;
			// Enable the USB common interrupts we care about: Reset, Resume, Suspend.
			// Without this, we USBCIF.SUSPENDIF will not get set (the datasheet is incomplete).
			USBCIE = 0b0111;

			// watchdog mode??? this will do a reset?
			//			WDCTL=0x0B;
			// delayMs(50);    //wait for reset
			// clear sent_beacon so we send it next time we wake up.
			do_sleep = 0;
			sent_beacon = 0;
			// power on the BLE module
			setDigitalOutput(10,HIGH);
		}
	}
}