/** xBridge2:
All thanks to Adrien de Croy, who created dexterity-wixel and modified the
wixel-sdk, upon which xBridge2 is based.  Without his efforts in this and decoding the
Dexcom protocol, this would not have been possible. 

If you are going to build from this source file, please ensure you are using
Adrien's wixel-sdk or it will not compile.

Thanks also to Stephen Black for his work on designing the bridge hardware based on
the wixel.  This code requires a minor modification to his design to enable bridge battery
monitoring.

This code could be modified to use a wixel with any BLE module.
It could likely also be modified to work with any CGM transmitter.

== Description ==
A program to allow a wixel to capture packets from a Dexcom G4 Platinum 
Continuous Glucose Monitor transmitter and send them in binary form
out of the UART1 port, and USB port if it is connected.

Note that debug messages are sent out the USB port only.

It also accepts various protocol packets on either the UART1 or USB ports, one of
which allows you to see or set the Dexcom Transmitter ID in the form of an encoded
long int (ie 63GEA).  It will NOT display packets
unless TXID is set.  Your application will need to accept a "beacon" packet from the
wixel, sent on both USB and UART0, and if the TXID value is zero, set it before the wixel
will send Dexcom packets from the specified Transmitter.


The app uses the radio_queue libray to receive packets.  It does not
transmit any packets.

The input to this app is mostly on thw radio reciever.  It also accepts an anlogue input on pin P0_0 (referenced to ground)
connected to any rechargeable battery being used to power the wixel, HM-10/11 etc.

The output from this app takes the following format:

The red LED indicates activity on the radio channel (packets being received).
Since every radio packet has a chance of being lost, there is no guarantee
that this app will pick up all the packets being sent.  However, it does a really
good job of detecting and filtering each packet if it is in range of the wixel.

Protocol descriptions:
Data Packet - Bridge to App.  Sends the Dexcom transmitter data, and the bridge battery volts.
	0x11	- length of packet.
	0x00	- Packet type (00 means data packet)
	uint32	- Dexcom Raw value.
	uint32	- Dexcom Filtered value.
	uint8	- Dexcom battery value.
	uint16	- Bridge battery value.
	uint32	- Dexcom encoded TXID the bridge is filtering on.
	
Data Acknowledge Packet - App to Bridge.  Sends an ack of the Data Packet and tells the wixel to go to sleep.
	0x02	- length of packet.
	0xF0	- Packet type (F0 means acknowleged, go to sleep.
	
TXID packet - App to Bridge.  Sends the TXID the App wants the bridge to filter on.  In response to a Data packet or beacon packet being wrong.
	0x06	- Length of the packet.
	0x01	- Packet Type (01 means TXID packet).
	uint32	- Dexcom encoded TXID.
	
Beacon Packet - Bridge to App.  Sends the TXID it is filtering on to the app, so it can set it if it is wrong.
				Sent when the wixel wakes up, or as acknowledgement of a TXID packet.
	0x06	- Length of the packet.
	0xF1	- Packet type (F1 means Beacon or TXID acknowledge)
	uint32	- Dexcom encoded TXID.
	
Note:  The protocol can be expanded simply by adding other packet types.  I envision that a vibration module could
be added, with a wixel output to drive it, so that the wixel will cause a vibration when the phone app sends a command.
This would act as a backup to the phone alerts, and perhaps smart watch alerts, in case a T1D has left his phone/watch
elsewhere.  This small bridge rig should be kept nearby the T1D at all times.
*/

/** Dependencies **************************************************************/
//#include <wixel.h>
#include <board.h>
#include <usb.h>
#include <usb_com.h>
#include <gpio.h>
#include <adc.h>
#include "time.h"

#include <radio_registers.h>
#include <radio_mac.h>
#include <random.h>
//#include <radio_queue.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <uart1.h>

//define the xBridge Version
#define VERSION ("2.40")
//define the FLASH_TX_ID address.  This is the address we store the Dexcom TX ID number in.
//#define FLASH_TX_ID		(0x77F8)
//define the DEXBRIDGE_FLAGS address.  This is the address we store the xBridge flags in.
//#define DEXBRIDGE_FLAGS	(FLASH_TX_ID-2)
//define the FLASH_BATTERY_MAX address.  This is the address we store the battery_maximum value in flash.
//#define FLASH_BATTERY_MAX (DEXBRIDGE_FLAGS-2)
//define the FLASH_BATTERY_MIN address.  This is the address we store the battery_maximum value in flash.
//#define FLASH_BATTERY_MIN (FLASH_BATTERY_MAX-2)
//define the maximum command string length for USB commands.
#define USB_COMMAND_MAXLEN	(32)
//defines the number of channels we will scan.
#define NUM_CHANNELS		(4)
//define the start channel
#define START_CHANNEL		(0)
//defines battery minimum and maximum voltage values for converting to percentage.
/* To calculate the value for a specific voltage, use the following formula
	val = (((voltage/RH+RL)*RL)/1.25)*2047
where:
	val is the value to enter into one of the limits.  Rounded to the nearest Integer.
	voltage is the voltage you want the value for.
	RH is the Larger resistor value in the voltage divider circuit (VIN to P0_0)
	RL is the lower resistor value in the voltage divider circuit (P0_0 to Ground)
*/
// assuming that there is a 27k ohm resistor between VIN and P0_0, and a 10k ohm resistor between P0_0 and GND, as per xBridge circuit diagrams
#define BATTERY_MAXIMUM		(1814) //4.1V
#define BATTERY_MINIMUM		(1416) //3.2V
//If using Chris Bothelo's bridge circuit, the code will automatically use these values instead.
#define BATTERY_MAXIMUM_CLASSIC (2034)
#define BATTERY_MINIMUM_CLASSIC (1587)
// defines the xBridge protocol functional level.  Sent in each packet as the last byte.
#define DEXBRIDGE_PROTO_LEVEL (0x01)
//define the Packet Receive Timeout on each channel in ms

#define wake_before_packet (40)         	// millyseconds to wake before a packet is expected
#define wake_wixel_beofre_packet (40)       // seconds to wake the BT system before a packet

static volatile BIT do_sleep = 0;		// indicates we should go to sleep between packets
static volatile BIT is_sleeping = 0;	        // flag indicating we are sleeping.
static volatile BIT usb_connected;		// indicates DTR set on USB.  Meaning a terminal program is connected via USB for debug.
static volatile BIT sent_beacon;		// indicates if we have sent our current dex_tx_id to the app.
static volatile BIT writing_flash;		// indicates if we are writing to flash.
static volatile BIT ble_sleeping;		// indicates if we have recieved a message from the BLE module that it is sleeping, or if false, it is awake.
static volatile BIT dex_tx_id_set;		// indicates if the Dexcom Transmitter id (settings.dex_tx_id) has been set.  Set in doServices.
static volatile BIT ble_connected;		// bit indicating the BLE module is connected to the phone.  Prevents us from sending data without this.
static volatile BIT save_settings;		// flag to indicate that settings should be saved to flash.
static volatile BIT got_packet;			// flag to indicate we have captured a packet.
static volatile BIT got_ok;				// flag indicating we got OK from the HM-1x
static volatile BIT do_leds;			// flag indicating to NOT show LEDs
static volatile BIT send_debug;			// flag indicating to send debug output
static volatile BIT sleep_ble;			// flag indicating to sleep the BLE module (power down in sleep)
static volatile uint32 dly_ms = 0;
static volatile uint32 pkt_time = 0;
// these flags will be stored in Flash
static volatile BIT initialised;					// flag to indicate we have passed the initialisation.
static uint8 sleep_mode = 0;
static uint8 save_IEN0;
static uint8 save_IEN1;
static uint8 save_IEN2;


XDATA uint8 battery_capacity=0;
XDATA uint32 last_beacon=0;
XDATA uint32 last_battery=0;
XDATA uint8 last_channel = 0; // last channel we captured a packet on
// _Dexcom_packet - Type Definition of a Dexcom Radio packet
// actual packet length is 17 bytes, excluding len and checksum, rssi &LQI.
typedef struct _Dexcom_packet
{
	uint8	len;
	uint32	dest_addr;
	uint32	src_addr;
	uint8	port;
	uint8	device_info;
	uint8	txId;
	uint16	raw;
	uint16	filtered;
	uint8	battery;
	uint8	unknown;
	uint8	checksum;
	int8	RSSI;
	uint8	LQI;
} Dexcom_packet;

XDATA Dexcom_packet * Pkt;

XDATA uint8 currentPacket[sizeof(Dexcom_packet)];

// _xBridge_settings - Type definition for storage of xBridge_settings
typedef struct _xBridge_settings
{
	uint32 dex_tx_id; 		//4 bytes
	uint16 battery_maximum;	//2 bytes
	uint16 battery_minimum;	//2 bytes
	uint16 flags;			//2 bytes
	uint32 uart_baudrate;	//4 bytes
} xBridge_settings;			//14 bytes total
XDATA xBridge_settings settings;

#define FLASH_SETTINGS 			(0x77E0)
#define BLE_INITIALISED			(0x0001)
#define SLEEP_BLE				(0x0002)
#define DONT_IGNORE_BLE_STATE	(0x0004)
#define XBRIDGE_HW				(0x0008)
#define DO_LEDS					(0x0010)
#define SEND_DEBUG				(0x0020)
// next flag will be 0x0040

// array of HM-1x baudrates for rate detection.
XDATA uint32 uart_baudrate[9] = {9600,19200,38400,57600,115200,4800,2400,1200,230400};

//structure of a USB command
typedef struct _command_buff
{
	uint8 commandBuffer[USB_COMMAND_MAXLEN];
	uint8 nCurReadPos;
} t_command_buff;

//create a static command structure to use.
static t_command_buff command_buff;

void setFlag(uint8 ptr, uint8 val)
{
	if(val)
	{
		settings.flags |= ptr;
	}
	else
	{
		settings.flags &= ~ptr;
	}
}

unsigned char getFlag(uint8 ptr)
{
	return settings.flags & ptr ? 1 : 0;
}

// Dexcom Transmitter Source Address to match against.
//XDATA uint32 dex_tx_id; 
// xBridge flags.  Currently only the following flags are used:
//	0	-	BLE name initialised. 1= not initialised, 0= initialised.
//XDATA uint16 xBridge_flags;
//XDATA uint16 battery_maximum = BATTERY_MAXIMUM;
//XDATA uint16 battery_minimum = BATTERY_MINIMUM;

// message  buffer for communicating with the BLE module
XDATA uint8 msg_buf[82];

// Initialization of source buffers and DMA descriptor for the DMA transfer during PM2 sleep.
unsigned char XDATA PM2_BUF[7] = {0x06,0x06,0x06,0x06,0x06,0x06,0x04};
//unsigned char XDATA PM3_BUF[7] = {0x07,0x07,0x07,0x07,0x07,0x07,0x04};
unsigned char XDATA dmaDesc[8] = {0x00,0x00,0xDF,0xBE,0x00,0x07,0x20,0x42};


// forward prototypes
// prototype for doServices function.
int doServices(uint8 bWithProtocol);
// prototype for getSrcValue function
uint32 getSrcValue(char srcVal);
// prototype for batteryPercent function
uint8 battteryPercent(uint32 val);
// prototype for sendBeacon function
void sendBeacon(void);


// frequency offsets for each channel - seed to 0.
static uint8 fOffset[NUM_CHANNELS] = {0xCE,0xD5,0xE6,0xE5};
static uint8 nChannels[NUM_CHANNELS] = { 0, 100, 199, 209 };


// writeBuffer holds the data from the computer that we want to write in to flash.
XDATA uint8 writeBuffer[sizeof(settings)];

// flashWriteDmaConfig holds the configuration of DMA channel 0, which we use to
// transfer the data from writeBuffer in to flash.
XDATA DMA_CONFIG flashWriteDmaConfig;



// startFlashWrite is a small piece of code we store in RAM which initiates the flash write.
// According to datasheet section 12.3.2.1, this code needs to be 2-aligned if it is executed
// from flash. SDCC does not have a good way of 2-aligning code, so we choose to put the code in
// RAM instead of flash.
XDATA uint8 startFlashWrite[] = {
	0x75, 0xAE, 0x02,  // mov _FCTL, #2 :   Sets FCTRL.WRITE bit to 1, initiating a write to flash.
	0x22               // ret           :   Returns to the calling function.
};
/* dma_init:  	A function to initialise the DMA channel for use in erasing and storing data in flash.
Parameters:		none.
Returns :		void.
Uses :			writeBuffer - A global variable we can address as the write buffer for writing data.
*/

/* radio queue functions and defines.  Replaces radio_queue library
Note:  These functions are taken directly from the radio_queue source code, and modified to remove the transmit functions.
This is probably NOT the best way to achieve this, but it is better than modifying the radio_queue library source.
*/
/* PARAMETERS *****************************************************************/

int32 CODE param_radio_channel = 128;

/* PACKET VARIABLES AND DEFINES ***********************************************/

// Compute the max size of on-the-air packets.  This value is stored in the PKTLEN register.
#define RADIO_QUEUE_PAYLOAD_SIZE	(19)
#define RADIO_MAX_PACKET_SIZE  (RADIO_QUEUE_PAYLOAD_SIZE)

#define RADIO_QUEUE_PACKET_LENGTH_OFFSET 0

/*  rxPackets:
 *  We need to be prepared at all times to receive a full packet from another
 *  party, even if we cannot give it to the main loop.  Therefore, we need (at
 *  least) THREE buffers, so that two can be owned by the main loop while
 *  another is owned by the ISR and ready to receive the next packet.
 *
 *  If a packet is received and the main loop still owns the other two buffers,
 *  we discard it.
 *
 *  Ownership of the RX packet buffers is determined from radioQueueRxMainLoopIndex and radioQueueRxInterruptIndex.
 *  The main loop owns all the buffers from radioQueueRxMainLoopIndex to radioQueueRxInterruptIndex-1 inclusive.
 *  If the two indices are equal, then the main loop owns nothing.  Here are three examples:
 *
 *  radioQueueRxMainLoopIndex | radioQueueRxInterruptIndex | Buffers owned by main loop.
 *                0 |                0 | None
 *                0 |                1 | rxBuffer[0]
 *                0 |                2 | rxBuffer[0 and 1]
 */
#define RX_PACKET_COUNT  3
static volatile uint8 XDATA radioQueueRxPacket[RX_PACKET_COUNT][1 + RADIO_MAX_PACKET_SIZE + 2];  // The first byte is the length.
static volatile uint8 DATA radioQueueRxMainLoopIndex = 0;   // The index of the next rxBuffer to read from the main loop.
static volatile uint8 DATA radioQueueRxInterruptIndex = 0;  // The index of the next rxBuffer to write to when a packet comes from the radio.

/* txPackets are handled similarly */
#define TX_PACKET_COUNT 16
static volatile uint8 XDATA radioQueueTxPacket[TX_PACKET_COUNT][1 + RADIO_MAX_PACKET_SIZE];  // The first byte is the length.
static volatile uint8 DATA radioQueueTxMainLoopIndex = 0;   // The index of the next txPacket to write to in the main loop.
static volatile uint8 DATA radioQueueTxInterruptIndex = 0;  // The index of the current txPacket we are trying to send on the radio.

//BIT radioQueueAllowCrcErrors = 0;
BIT radioQueueAllowCrcErrors = 1;

/* GENERAL FUNCTIONS **********************************************************/

void radioQueueInit()
{
    randomSeedFromSerialNumber();

    PKTLEN = RADIO_MAX_PACKET_SIZE;
    CHANNR = param_radio_channel;

    radioMacInit();
    radioMacStrobe();
}

// Returns a random delay in units of 0.922 ms (the same units of radioMacRx).
// This is used to decide when to next transmit a queued data packet.
static uint8 randomTxDelay()
{
    //return 1 + (randomNumber() & 3);  //original radio_queue value drops a lot of packets ~= 1-4ms delay
    //return 4 + (randomNumber() & 3);  //first try at lengthening.  Works really well.  ~= 4-7ms delay
    //return 6 + (randomNumber() & 3);	//next try.  Drops around the same as 1.  ~= 6-9 ms delay
    //return 5 + (randomNumber() & 3);	//next try.  Still not as good as 4, but way better than 6  ~= 5-8ms delay
	return 3 + (randomNumber() & 3);	//next try. ~= 3-6ms delay
}

/* TX FUNCTIONS (called by higher-level code in main loop) ********************/
/*
uint8 radioQueueTxAvailable(void)
{
    // Assumption: TX_PACKET_COUNT is a power of 2
    return (radioQueueTxInterruptIndex - radioQueueTxMainLoopIndex - 1) & (TX_PACKET_COUNT - 1);
}

uint8 radioQueueTxQueued(void)
{
    return (radioQueueTxMainLoopIndex - radioQueueTxInterruptIndex) & (TX_PACKET_COUNT - 1);
}

uint8 XDATA * radioQueueTxCurrentPacket()
{
    if (!radioQueueTxAvailable())
    {
        return 0;
    }

    return radioQueueTxPacket[radioQueueTxMainLoopIndex];
}

void radioQueueTxSendPacket(void)
{
    // Update our index of which packet to populate in the main loop.
    if (radioQueueTxMainLoopIndex == TX_PACKET_COUNT - 1)
    {
        radioQueueTxMainLoopIndex = 0;
    }
    else
    {
        radioQueueTxMainLoopIndex++;
    }

    // Make sure that radioMacEventHandler runs soon so it can see this new data and send it.
    // This must be done LAST.
    radioMacStrobe();
}
*/
/* RX FUNCTIONS (called by higher-level code in main loop) ********************/

uint8 XDATA * radioQueueRxCurrentPacket(void)
{
    if (radioQueueRxMainLoopIndex == radioQueueRxInterruptIndex)
    {
        return 0;
    }
    return radioQueueRxPacket[radioQueueRxMainLoopIndex];
}

void radioQueueRxDoneWithPacket(void)
{
    if (radioQueueRxMainLoopIndex == RX_PACKET_COUNT - 1)
    {
        radioQueueRxMainLoopIndex = 0;
    }
    else
    {
        radioQueueRxMainLoopIndex++;
    }
}

/* FUNCTIONS CALLED IN RF_ISR *************************************************/

static void takeInitiative()
{
    if (radioQueueTxInterruptIndex != radioQueueTxMainLoopIndex)
    {
        // Try to send the next data packet.
        radioMacTx(radioQueueTxPacket[radioQueueTxInterruptIndex]);
    }
    else
    {
        radioMacRx(radioQueueRxPacket[radioQueueRxInterruptIndex], 0);
    }
}

void radioMacEventHandler(uint8 event) // called by the MAC in an ISR
{
    if (event == RADIO_MAC_EVENT_STROBE)
    {
        takeInitiative();
        return;
    }
  /*  else if (event == RADIO_MAC_EVENT_TX)
    {
        // Give ownership of the current TX packet back to the main loop by updated radioQueueTxInterruptIndex.
        if (radioQueueTxInterruptIndex == TX_PACKET_COUNT - 1)
        {
            radioQueueTxInterruptIndex = 0;
        }
        else
        {
            radioQueueTxInterruptIndex++;
        }

        // We sent a packet, so now let's give another party a chance to talk.
        radioMacRx(radioQueueRxPacket[radioQueueRxInterruptIndex], randomTxDelay());
        return;
    }
*/    else if (event == RADIO_MAC_EVENT_RX)
    {
        uint8 XDATA * currentRxPacket = radioQueueRxPacket[radioQueueRxInterruptIndex];

        if (!radioQueueAllowCrcErrors && !radioCrcPassed())
        {
            if (radioQueueTxInterruptIndex != radioQueueTxMainLoopIndex)
            {
                radioMacRx(currentRxPacket, randomTxDelay());
            }
            else
            {
                radioMacRx(currentRxPacket, 0);
            }
            return;
        }

        if (currentRxPacket[RADIO_QUEUE_PACKET_LENGTH_OFFSET] > 0)
        {
            // We received a packet that contains actual data.

            uint8 nextradioQueueRxInterruptIndex;

            // See if we can give the data to the main loop.
            if (radioQueueRxInterruptIndex == RX_PACKET_COUNT - 1)
            {
                nextradioQueueRxInterruptIndex = 0;
            }
            else
            {
                nextradioQueueRxInterruptIndex = radioQueueRxInterruptIndex + 1;
            }

            if (nextradioQueueRxInterruptIndex != radioQueueRxMainLoopIndex)
            {
                // We can accept this packet!
                radioQueueRxInterruptIndex = nextradioQueueRxInterruptIndex;
            }
        }

        takeInitiative();
        return;
    }
    else if (event == RADIO_MAC_EVENT_RX_TIMEOUT)
    {
        takeInitiative();
        return;
    }
}
/* End of radio_queue functions *********************************************************************/
/* time functions.  Replaces the time library. 
Note:  These functions and variables are taken direct from the time library, and have been slightly modified
and added to.  This is preferable to modifying the time library source code directly.
*/
PDATA volatile uint32 timeMs;
//PDATA volatile uint32 time_since_pkt;

ISR(T4, 0)
{
    timeMs++;
    // T4CC0 ^= 1; // If we do this, then on average the interrupts will occur precisely 1.000 ms apart.
}

uint32 getMs()
{
    uint8 oldT4IE = T4IE;   // store state of timer 4 interrupt (active/inactive?)
    uint32 time;
    T4IE = 0;               // disable timer 4 interrupt
    time = timeMs;          // copy millisecond timer count into a safe variable
    T4IE = oldT4IE;         // restore timer 4 interrupt to its original state
    return time;            // return timer count copy
}

/* addMs - This is an added function to allow the timeMs variable to be added to
We use this in the code to append the sleep time to the timeMs variable so we can keep track of
precisely how long it has been since the last packet was received, even during sleep.
*/
void addMs(uint32 addendum)
{
	uint8 oldT4IE = T4IE;	// store state of timer 4 interrupt (active/inactive?)
	T4IE = 0;				// disable timer 4 interrupt
	timeMs += addendum; 	// add addendum to timeMs
	T4IE = oldT4IE;			// restore timer 4 interrupt to it's original state
}

void timeInit()
{
    T4CC0 = 187;
    T4IE = 1;     // Enable Timer 4 interrupt.  (IEN1.T4IE=1)

    // DIV=111: 1:128 prescaler
    // START=1: Start the timer
    // OVFIM=1: Enable the overflow interrupt.
    // MODE=10: Modulo
    T4CTL = 0b11111010;

    EA = 1; // Globally enable interrupts (IEN0.EA=1).
}

void delayMs(uint16 milliseconds)
{
    // TODO: make this more accurate.
    // A great way would be to use the compare feature of Timer 4 and then
    // wait for the right number of compare events to happen, but then we
    // can't use that channel for PWM in the future.
    while(milliseconds--)
    {
        delayMicroseconds(250);
        delayMicroseconds(250);
        delayMicroseconds(250);
        delayMicroseconds(249); // there's some overhead, so only delay by 249 here
    }
}

/* end time functions ***********************************************************/

/* Flash memory functions and variables.
All of these are to do with erasing and writing data into Flash memory.  Used to 
store the settings of xBridge.
*/
 void dma_Init(){
	// Configure the flash write timer.  See section 12.3.5 of the datasheet.
	FWT = 32;

	// Set up the DMA configuration block we use for writing to flash (See Figure 21 of the datasheet).
	// LENL and LENH are sent later when we know how much data to write.
	flashWriteDmaConfig.SRCADDRH = (unsigned short)&writeBuffer >> 8;
	flashWriteDmaConfig.SRCADDRL = (unsigned char)&writeBuffer;
	flashWriteDmaConfig.DESTADDRH = XDATA_SFR_ADDRESS(FWDATA) >> 8;
	flashWriteDmaConfig.DESTADDRL = XDATA_SFR_ADDRESS(FWDATA);

	// WORDSIZE = 0     : byte
	// TMODE = 0        : single mode
	// TRIG = 0d18      : flash
	flashWriteDmaConfig.DC6 = 18;

	// SRCINC = 01      : yes
	// DESTINC = 00     : no
	// IRQMASK = 0      : no   *datasheet said this bit should be 1
	// M8 = 0           : 8-bit transfer
	// PRIORITY = 0b10  : high
	flashWriteDmaConfig.DC7 = 0b01000010;

	DMA0CFG = (uint16)&flashWriteDmaConfig;
}
/* eraseFlash:	A function to erase a page of flash.
Note:  This erases a full page of flash, not just the data at the address you specify.
		It works out which page the address you speicfy is in, and erases that page.
		Flash MUST be erased (set every bit in every byte of the flasn page to 1) before
		you can change the data by writing to it.
Parameters:
	uint16	address		Address of the data that you want erased.  Read the note above.
Returns:	void
Uses:	The DMA channel initialised previously.
*/
void eraseFlash(uint16 address)
{
	// first erase the page
    FADDRH  = address >> 9;	// high byte of address / 2
    FADDRL =0;
    FCTL = 1;				// Set FCTL.ERASE to 1 to initiate the erasing.
    __asm nop __endasm;		// Datasheet says a NOP is necessary after the instruction that initiates the erase.
    __asm nop __endasm;		// We have extra NOPs to be safe.
    __asm nop __endasm;
    __asm nop __endasm;
    while(FCTL & 0x80){};	// Wait for erasing to be complete.
}

/* writeToFlash:	A function to write a value into flash.
Note:  This writes writebuffer to the specified flash address.
Parameters:
	uint16	address		Address of the data that you want erased.  Read the note above.
	uint16	length		The length of the data to write.  Basically the amount of data in writeBuffer.
Returns:	void
Uses:	The DMA channel initialised previously.
	global writeBuffer	Stores the data to be written to flash.
*/
void writeToFlash(uint16 address, uint16 length)
{
	FADDR = address >> 1;	// not sure if i need to do this again.
	flashWriteDmaConfig.VLEN_LENH = length >> 8;
	flashWriteDmaConfig.LENL = length;
	DMAIRQ &= ~(1<<0);		// Clear DMAIF0 so we can poll it to see when the transfer finishes.
	DMAARM |= (1<<0);
	__asm lcall _startFlashWrite __endasm;
	while(!(DMAIRQ & (1<<0))){}	// wait for the transfer to finish by polling DMAIF0
	while(FCTL & 0xC0){}		// wait for last word to finish writing by polling BUSY and SWBUSY
}

// Function to save all settings to flash
void saveSettingsToFlash()
{
		writing_flash=1;
		memcpy(&writeBuffer, &settings, sizeof(settings));
		eraseFlash(FLASH_SETTINGS);
		writeToFlash(FLASH_SETTINGS, sizeof(settings));
		writing_flash=0;
		save_settings=0;
		if(send_debug)
			printf_fast("settings saved to flash\r\n");
} 


// store RF config in FLASH, and retrieve it from here to put it in proper location (also incidentally in flash).
// this allows persistent storage of RF params that will survive a restart of the wixel (although not a reload of wixel app obviously).
// TO-DO get this working with DMA - need to erase memory block first, then write this to it.
//uint8 XDATA RF_Params[50];


void LoadRFParam(unsigned char XDATA* addr, uint8 default_val)
{
		*addr = default_val; 
}


/*
void uartDisable() {
    U1UCR &= ~0x40; //Hardware Flow Control (CTS/RTS) Off.  We always want it off.
//    U1CSR &= ~0x40; // Recevier disable.
}
*/

void dex_RadioSettings()
{
    // Transmit power: one of the highest settings, but not the highest.
    LoadRFParam(&PA_TABLE0, 0x00);

    // Set the center frequency of channel 0 to 2403.47 MHz.
    // Freq = 24/2^16*(0xFREQ) = 2403.47 MHz
    // FREQ[23:0] = 2^16*(fCarrier/fRef) = 2^16*(2400.156/24) = 0x6401AA
    //IOCFG2 = 0x0E; //RX_SYMBOL_TICK
    //IOCFG1 = 0x16; //RX_HARD_DATA[1]
    //IOCFG0 = 0x1D; //Preamble Quality Reached
    LoadRFParam(&IOCFG0, 0x0E);
    LoadRFParam(&FREQ2, 0x65);
    LoadRFParam(&FREQ1, 0x0A);
    LoadRFParam(&FREQ0, 0xAA);
    LoadRFParam(&SYNC1, 0xD3);
    LoadRFParam(&SYNC0, 0x91);
    LoadRFParam(&ADDR, 0x00);
    
    // Controls the FREQ_IF used for RX.
    // This is affected by MDMCFG2.DEM_DCFILT_OFF according to p.212 of datasheet.
    LoadRFParam(&FSCTRL1, 0x0A);				// Intermediate Freq.  Fif = FRef x FREQ_IF / 2^10 = 24000000 * 10/1024 = 234375  for 0x0F = 351562.5
    LoadRFParam(&FSCTRL0, 0x00);				// base freq offset.  This is 1/214 th of the allowable range of frequency compensation
												// which depends on the FOCCFG param for fraction of channel bandwidth to swing (currently 1/8th, maybe should be 1/4).
												
    // Sets the data rate (symbol rate) used in TX and RX.  See Sec 13.5 of the datasheet.
    // Also sets the channel bandwidth.
    // We tried different data rates: 375 kbps was pretty good, but 400 kbps and above caused lots of packet errors.
    // NOTE: If you change this, you must change RSSI_OFFSET in radio_registers.h


	// Dexcom states channel bandwidth (not spacing) = 334.7 kHz E = 1, M = 0 (MDMCFG4 = 4B)
	// =        24000000 / 8 x (4 + 0) x 2 ^ 1
	// =        24000000 / 64 = 375000
    LoadRFParam(&MDMCFG4, 0x4B);				// 375kHz BW, DRATE_EXP = 11.  
	// Rdata = (256+DRATE_M) x 2 ^ DRATE_E
	//           ------------------------ x Fref = FRef x (256 + DRATE_M) x 2 ^ (DRATE_E-28)
	//                2 ^ 28
	// in our case = 24000000 * (256+17) x 2 ^ (-17) = (24000000 / 131072) * 273 = 49987.79 b/s
	LoadRFParam(&MDMCFG3, 0x11);				// DRATE_M = 0x11 = 17.

    // MDMCFG2.DEM_DCFILT_OFF, 0, enable digital DC blocking filter before
    //   demodulator.  This affects the FREQ_IF according to p.212 of datasheet.
    // MDMCFC2.MANCHESTER_EN, 0 is required because we are using MSK (see Sec 13.9.2)
    // MDMCFG2.MOD_FORMAT, 111: MSK modulation
    // MDMCFG2.SYNC_MODE, 011: 30/32 sync bits received, no requirement on Carrier sense
    LoadRFParam(&MDMCFG2, 0x73);

    // MDMCFG1.FEC_EN = 0 : 0=Disable Forward Error Correction
    // MDMCFG1.NUM_PREAMBLE = 000 : Minimum number of preamble bytes is 2.
    // MDMCFG1.CHANSPC_E = 3 : Channel spacing exponent.
    // MDMCFG0.CHANSPC_M = 0x55 : Channel spacing mantissa.
    // Channel spacing = (256 + CHANSPC_M)*2^(CHANSPC_E) * f_ref / 2^18
	// = 24000000 x (341) x 2^(3-18) = 24000000 x 341 / 2^15
	// = 249755Hz.
    LoadRFParam(&MDMCFG1, 0x03);	// no FEC, preamble bytes = 2 (AAAA), CHANSPC_E = 3
    LoadRFParam(&MDMCFG0, 0x55);	// CHANSPC_M = 0x55 = 85


    LoadRFParam(&DEVIATN, 0x00);
    // See Sec 13.9.2.

    LoadRFParam(&FREND1, 0xB6);
    LoadRFParam(&FREND0, 0x10);

    // F0CFG and BSCFG configure details of the PID loop used to correct the
    // bit rate and frequency of the signal (RX only I believe).
    //LoadRFParam(&FOCCFG, 0x0A);		// allow range of +/1 FChan/4 = 375000/4 = 93750.  No CS GATE
	LoadRFParam(&FOCCFG, 0x2A);		// allow range of +/1 FChan/4 = 375000/4 = 93750.  With CS GATE
	
    LoadRFParam(&BSCFG, 0x6C);

    // AGC Control:
    // This affects many things, including:
    //    Carrier Sense Absolute Threshold (Sec 13.10.5).
    //    Carrier Sense Relative Threshold (Sec 13.10.6).
    LoadRFParam(&AGCCTRL2, 0x44);
    //LoadRFParam(&AGCCTRL1, 0x00);
    LoadRFParam(&AGCCTRL1, 0x50);	//enable relative Carrier Sense of 6db change
	LoadRFParam(&AGCCTRL0, 0xB2);

    // Frequency Synthesizer registers that are not fully documented.
    LoadRFParam(&FSCAL3, 0xA9); 
    LoadRFParam(&FSCAL2, 0x0A); 
    LoadRFParam(&FSCAL1, 0x20);
    LoadRFParam(&FSCAL0, 0x0D);

    // Mostly-undocumented test settings.
    // NOTE: The datasheet says TEST1 must be 0x31, but SmartRF Studio recommends 0x11.
    LoadRFParam(&TEST2, 0x81);
    LoadRFParam(&TEST1, 0x35);
    LoadRFParam(&TEST0, 0x0B);

    // Packet control settings.
    LoadRFParam(&PKTCTRL1, 0x04); 
    LoadRFParam(&PKTCTRL0, 0x05);		// enable CRC flagging and variable length packets.  Probably could use fix length for our case, since all are same length.
										// but that would require changing the library code, since it sets up buffers etc etc, and I'm too lazy.
	LoadRFParam(&PKTLEN, 0x12);			// limit packet length to 18 bytes, which should be the length of the Dexcom packet.
	
	//RF_Params[49] = 1;
}


uint8 min8(uint8 a, uint8 b)
{
	if(a < b) return a;
	return b;
}


/*
// getPacketRSSI - returns the RSSI value from the Dexcom_packet passed to it.
int8 getPacketRSSI(Dexcom_packet* p)
{
	// RSSI_OFFSET for 489 kbaud is closer to 73 than 71
	return (p->RSSI/2)-73;
}


//getPacketPassedChecksum - returns the checksum of the Dexcom_packet passed to it.
uint8 getPacketPassedChecksum(Dexcom_packet* p)
{
	//take the LQI value from the packet, AND with 0x80.  If it equals 0x80, then return 1, otherwise return 0.
	return ((p->LQI & 0x80)==0x80) ? 1:0;
}
*/
//format an array to decode the dexcom transmitter name from a Dexcom packet source address.
XDATA char SrcNameTable[32] = { '0', '1', '2', '3', '4', '5', '6', '7',
						  '8', '9', 'A', 'B', 'C', 'D', 'E', 'F',
						  'G', 'H', 'J', 'K', 'L', 'M', 'N', 'P',
						  'Q', 'R', 'S', 'T', 'U', 'W', 'X', 'Y' };

// convert the passed uint32 Dexcom source address into an ascii string in the passed char addr[6] array.
char *dexcom_src_to_ascii(uint32 src)
{
	XDATA char addr[6];
	//each src value is 5 bits long, and is converted in this way.
	addr[0] = SrcNameTable[(src >> 20) & 0x1F];		//the last character is the src, shifted right 20 places, ANDED with 0x1F
	addr[1] = SrcNameTable[(src >> 15) & 0x1F];		//etc
	addr[2] = SrcNameTable[(src >> 10) & 0x1F];		//etc
	addr[3] = SrcNameTable[(src >> 5) & 0x1F];		//etc
	addr[4] = SrcNameTable[(src >> 0) & 0x1F];		//etc
	addr[5] = 0;	//end the string with a null character.
	return (char *)addr;
}


void uartEnable() {
    U1UCR &= ~0x40; //Hardware Flow Control (CTS/RTS) Off.  We always want it off.
    U1CSR |= 0x40; // Recevier enable
}

// function to wait a specified number of milliseconds, whilst processing services.
void waitDoingServices (uint32 wait_time, volatile BIT break_flag, BIT bProtocolServices ) {
	XDATA uint32 start_wait;
	start_wait = getMs();
	while ((getMs() - start_wait) < wait_time) {
		doServices(bProtocolServices);
		if(break_flag) return;
		delayMs(20);
	}
}




/** Functions *****************************************************************/
/* the functions that puts the system to sleep (PM2) and configures sleep timer to
wake it again in 250 seconds.*/
void makeAllOutputs(BIT value)
{
	//we only make the P1_ports low, and not P1_2 or P1_3
    int i;
    for (i=11;i < 17; i++)
	{
		if( i == 10 && !(sleep_ble))
			continue;
		setDigitalOutput(i, value);
    }
}

void sleepInit(void)
{
   WORIRQ  |= (1<<4); // Enable Event0 interrupt  
}

ISR(ST, 1)
{
   // Clear IRCON.STIF (Sleep Timer CPU interrupt flag)
   //IRCON &= 0x7F;
   IRCON &= 0x3F;
   // Clear WORIRQ.EVENT0_FLAG (Sleep Timer peripheral interrupt flag)
   // This is required for the CC111xFx/CC251xFx only!
   WORIRQ &= 0xFE;
   
   SLEEP &= 0xFC; // Not required when resuming from PM0; Clear SLEEP.MODE[1:0]
}

void switchToRCOSC(void)
{
   // Power up [HS RCOSC] (SLEEP.OSC_PD = 0)
   SLEEP &= ~0x04;
   // Wait until [HS RCOSC] is stable (SLEEP.HFRC_STB = 1)
   while ( ! (SLEEP & 0x20) );
   // Switch system clock source to HS RCOSC (CLKCON.OSC = 1),
   // and set max CPU clock speed (CLKCON.CLKSPD = 1).
   CLKCON = (CLKCON & ~0x07) | 0x40 | 0x01;
   // Wait until system clock source has actually changed (CLKCON.OSC = 1)
   while ( !(CLKCON & 0x40) );
   // Power down [HS XOSC] (SLEEP.OSC_PD = 1)
   SLEEP |= 0x04;
}




uint32 calcSleep(uint16 seconds) {
	uint32 diff = 0;
	XDATA uint32 now = 0;
	diff = seconds;
//	printf_fast("\r\ndiff: %lu\r\n", diff);
	diff = diff * 1000;
	now = diff;
	//printf_fast("\r\ndiff * 1000: %lu\r\n", diff);
	diff = diff - (getMs() - pkt_time);
//	printf_fast("\r\ndiff - getMs-pkt_time: %lu\r\n", diff);
//	diff = diff/1000;
//	while(diff>seconds)
//		diff-= 300;
	while(diff > now)
		diff = diff - now;
//	printf_fast("\r\ndiff: %lu\r\n", diff);
	return diff;
}

void goToSleep (uint16 seconds) {
    unsigned char temp;
	//uint16 sleep_time = 0;
	unsigned short sleep_time = 0;
	uint32 sleep_time_ms = 0;
	XDATA uint32 addendum = 0;
	//uint32 diff = 0;
	//uint32 now = 0;
	//initialise sleep library
	sleepInit();

    // The wixel docs note that any high output pins consume ~30uA
    makeAllOutputs(LOW);
	//printf_fast("\r\nseconds: %u, sleep_time: %u, now-pkt_time: %lu\r\n", seconds, sleep_time, (getMs()-pkt_time));
	//sleep_time_ms = calcSleep(seconds);
	while(usb_connected && (usbComTxAvailable() < 128)) {
		usbComService();
	}
	is_sleeping = 1;

	if(!usb_connected)
	{
		unsigned char storedDescHigh, storedDescLow;
		BIT	storedDma0Armed;
		unsigned char storedIEN0, storedIEN1, storedIEN2;
		disableUsbPullup();
		usbDeviceState = USB_STATE_DETACHED;
		// disable the USB module
		SLEEP &= ~(1<<7); // Disable the USB module (SLEEP.USB_EN = 0).
		// sleep power mode 2 is incompatible with USB - as USB registers lose state in this mode.

   
		//desired_event0 = seconds;
		
		// set Sleep Timer to the lowest resolution (1 second)      
		WORCTRL |= 0x03; 
		// must be using RC OSC before going to PM2
		switchToRCOSC();
		
		// Following DMA code is a workaround for a bug described in Design Note
		// DN106 section 4.1.4 where there is a small chance that the sleep mode
		// bits are faulty set to a value other than zero and this prevents the
		// processor from waking up correctly (appears to hang)
		
		// Store current DMA channel 0 descriptor and abort any ongoing transfers,
		// if the channel is in use.
		storedDescHigh = DMA0CFGH;
		storedDescLow = DMA0CFGL;
		storedDma0Armed = DMAARM & 0x01;
		DMAARM |= 0x81; // Abort transfers on DMA Channel 0; Set ABORT and DMAARM0
		// Update descriptor with correct source.
		dmaDesc[0] = ((unsigned int)& PM2_BUF) >> 8;
		dmaDesc[1] = (unsigned int)& PM2_BUF;
		// Associate the descriptor with DMA channel 0 and arm the DMA channel
		DMA0CFGH = ((unsigned int)&dmaDesc) >> 8;
		DMA0CFGL = (unsigned int)&dmaDesc;
		DMAARM = 0x01; // Arm Channel 0; DMAARM0
		
		// save enabled interrupts
		storedIEN0 = IEN0;
		storedIEN1 = IEN1;
		storedIEN2 = IEN2; 
		
		// make sure interrupts aren't completely disabled
		// and enable sleep timer interrupt
		IEN0 |= 0xA0; // Set EA and STIE bits
         
		// then disable all interrupts except the sleep timer
		IEN0 &= 0xA0;
		IEN1 &= ~0x3F;
		IEN2 &= ~0x3F;
		
		sleep_time_ms = calcSleep(seconds);
		sleep_time = (unsigned short)(sleep_time_ms/1000);
		//printf_fast("sleep_time: %u\r\n", sleep_time);
		//pkt_time += sleep_time_ms;
		if (sleep_time == 0 || sleep_time > seconds) {
			if(send_debug)
				printf_fast("too late to sleep, cancelling\r\n");
			//LED_YELLOW(1);
			// Switch back to high speed
			boardClockInit();   
			return;
		}
		WORCTRL |= 0x04; // Reset Sleep Timer, set resolution to 1 clock cycle
		temp = WORTIME0;
		while(temp == WORTIME0); // Wait until a positive 32 kHz edge
		temp = WORTIME0;
		while(temp == WORTIME0); // Wait until a positive 32 kHz edge
		WOREVT1 = sleep_time >> 8; // Set EVENT0, high byte
		WOREVT0 = sleep_time; // Set EVENT0, low byte
		MEMCTR |= 0x02;  // Flash cache must be disabled.
		SLEEP = 0x06; // PM2, disable USB, power down other oscillators
		
		__asm nop __endasm; 
		__asm nop __endasm; 
		__asm nop __endasm;
		
		if (SLEEP & 0x03) {
			__asm mov 0xD7,#0x01 __endasm; // DMAREQ = 0x01;
			__asm nop __endasm;            // Needed to perfectly align the DMA transfer.
			__asm orl 0x87,#0x01 __endasm; // PCON |= 0x01;
			__asm nop __endasm;      
		}
		// restore enabled interrupts
		IEN0 = storedIEN0;
		IEN1 = storedIEN1;
		IEN2 = storedIEN2; 
		// restore DMA descriptor
		DMA0CFGH = storedDescHigh;
		DMA0CFGL = storedDescLow;
		if (storedDma0Armed)
			DMAARM |= 0x01; // Set DMA0ARM
   
		// Switch back to high speed
		boardClockInit();   
		// add the time we were asleep to ms count
		addMs(sleep_time_ms);

	} else {
		// set Sleep Timer to the lowest resolution (1 second)      
		WORCTRL |= 0x03; // WOR_RES[1:0]
		// make sure interrupts aren't completely disabled
		// and enable sleep timer interrupt
		IEN0 |= 0xA0; // Set EA and STIE bits
       
		WORCTRL |= 0x04; // Reset Sleep Timer; WOR_RESET, and set resolution to 1 clock period
		temp = WORTIME0;
		while(temp == WORTIME0); // Wait until a positive 32 kHz edge
		temp = WORTIME0;
		while(temp == WORTIME0); // Wait until a positive 32 kHz edge

		sleep_time_ms = calcSleep(seconds);
		sleep_time = (unsigned short)(sleep_time_ms/1000);
		//printf_fast("sleep_time_ms: %lu, sleep_time: %u\r\n", sleep_time_ms, sleep_time);
		//pkt_time += sleep_time_ms;
		if (sleep_time == 0 || sleep_time > seconds) {
			//LED_YELLOW(1);
			// Switch back to high speed
			boardClockInit();   
			return;
		}
		WOREVT1 = sleep_time >> 8; // Set EVENT0, high byte
		WOREVT0 = sleep_time; // Set EVENT0, low byte

		// Set SLEEP.MODE according to PM1

		SLEEP = (SLEEP & 0xFC) | 0x01; // SLEEP.MODE[1:0]
		// Apply three NOPs to allow the corresponding interrupt blocking to take
		// effect, before verifying the SLEEP.MODE bits below. Note that all
		// interrupts are blocked when SLEEP.MODE ? 0, thus the time between
		// setting SLEEP.MODE ? 0, and asserting PCON.IDLE should be as short as
		// possible. If an interrupt occurs before the NOPs have completed, then
		// the enabled ISR shall clear the SLEEP.MODE bits, according to the code
		// in Figure 7.

		__asm nop __endasm;
		__asm nop __endasm;
		__asm nop __endasm;

		// If no interrupt was executed in between the above NOPs, then all
		// interrupts are effectively blocked when reaching this code position.
		// If the SLEEP.MODE bits have been cleared at this point, which means
		// that an ISR has indeed executed in between the above NOPs, then the
		// application will not enter PM{1 – 3} !
   
		if (SLEEP & 0x03) // SLEEP.MODE[1:0]
		{
			// Set PCON.IDLE to enter the selected PM, e.g. PM1.
			PCON |= 0x01;
			// The SoC is now in PM and will only wake up upon Sleep Timer interrupt
			// or external Port interrupt.
			__asm nop __endasm;    
		}
		// Switch back to high speed      
		boardClockInit(); 
		// add the time we were asleep to ms count
		addMs(sleep_time_ms);
	}
//	printf_fast("awake!  getMs is %lu\r\n", getMs());
//	printf_fast("slept for %lu us, %u s \r\n", sleep_time_ms, sleep_time);
	is_sleeping = 0;
}

void updateLeds()
{
	if (do_sleep)
	{
		if(is_sleeping && do_leds)
		{
			LED_YELLOW((getMs()&0x00000F00) == 0x100);
		}
	} 
	else 
	{
		if(getFlag(SLEEP_BLE)){
			if (do_leds) {
				LED_YELLOW(ble_connected);
			}
			else {
				LED_YELLOW(0);
			}
		}
		else {
			LED_YELLOW(0);
		}
		if(dex_tx_id_set)
		{
			if(do_leds) LED_RED(radioQueueRxCurrentPacket());
		} 
		else 
		{
			if(do_leds) LED_RED((getMs() & 0x0000FF00) == 0x1300);
		}
	}
}

// This is called by printf and printPacket.
void putchar(char c)
{
//	uart1TxSendByte(c);
	if (usb_connected)
		usbComTxSendByte(c);
}


//return a byte reversed from that passed.
uint8 bit_reverse_byte(uint8 in)
{
	uint8 bRet = 0;
	if(in & 0x01)
		bRet |= 0x80;
	if(in & 0x02)
		bRet |= 0x40;
	if(in & 0x04)
		bRet |= 0x20;
	if(in & 0x08)
		bRet |= 0x10;
	if(in & 0x10)
		bRet |= 0x08;
	if(in & 0x20)
		bRet |= 0x04;
	if(in & 0x40)
		bRet |= 0x02;
	if(in & 0x80)
		bRet |= 0x01;
	return bRet;
}
//reverse a byte buffer
void bit_reverse_bytes(uint8* buf, uint8 nLen)
{
	uint8 i = 0;
	for(; i < nLen; i++)
	{
		buf[i] = bit_reverse_byte(buf[i]);
	}
}

//decode the dex number (unsigned short float) as an unsigned 32bit integer.
uint32 dex_num_decoder(uint16 usShortFloat)
{
	uint16 usReversed = usShortFloat;
	uint8 usExponent = 0;
	uint32 usMantissa = 0;
	bit_reverse_bytes((uint8*)&usReversed, 2);
	usExponent = ((usReversed & 0xE000) >> 13);
	usMantissa = (usReversed & 0x1FFF);
	return usMantissa << usExponent;
}


void send_data( uint8 *msg, uint8 len)
{
	uint8 i = 0;
	//wait until uart1 Tx Buffer is empty
	while(uart1TxAvailable() < len) {};
	for(i=0; i < len; i++)
	{
		uart1TxSendByte(msg[i]);
	}
	while(uart1TxAvailable()<255) waitDoingServices(20,0,1);
	if(usb_connected) {
		if(send_debug)
			printf_fast("Sending: ");
		while(usbComTxAvailable() < len) {};
		for(i=0; i < len; i++)
		{
			usbComTxSendByte(msg[i]);
		}
		while(usbComTxAvailable()<128) waitDoingServices(20,0,1);
		if(send_debug)
			printf_fast("\r\nResponse: ");
	}
}


// function called by doServices to monitor the state of the BLE connection
void bleConnectMonitor() {
	//to store the time we went high
	static uint32 timer;
	//to store P1_2 the last time we looked.
	static BIT last_check;
	if(!initialised)
		return;
	if(!(getFlag(DONT_IGNORE_BLE_STATE)))
	{
		ble_connected = 1;
		sent_beacon = 0;
		return;
	}
	// if P1_2 is high, ble_connected is low, and the last_check was low, sav the time and set last_check.
	if (P1_2 && !ble_connected && !last_check) {
		timer=getMs();
		last_check = 1;
	// otherwise if P1_2 goes low, and ble_connected is high, we cancel everything.
	} else if (!P1_2) {
		ble_connected =0;
		last_check = 0;
		ble_connected = 0;
	//otherwise, if P1_2 has been high for more than 550ms, we can safely assume we have ble_connected, so say so.
	} else if (P1_2 && last_check && ((getMs() - timer)>550)) {
		ble_connected = 1;
		sent_beacon = 0;
	}
}
// Send a pulse to the BlueTooth module SYS input
/*void breakBt() {
	//pulse P1_2 high
	setDigitalOutput(12,HIGH);
	LED_RED(1);
	delayMs(200);
	//send P1_2 low
	setDigitalOutput(12,LOW);
}*/

// Configure the BlueTooth module with a name.
void configBt() {
	XDATA uint8 length;
	//uartEnable();
//	breakBt();
/*	length = sprintf(msg_buf,"AT+RENEW");
	send_data(msg_buf,length);
	delayMs(5000);
	length = sprintf(msg_buf,"AT+PWRM1");
	send_data(msg_buf,length);
	delayMs(1000);
*/	length = sprintf(msg_buf, "AT+NAMExBridge%02x%02x", serialNumber[0],serialNumber[1]);
    send_data(msg_buf, length);
	waitDoingServices(500,0,1);
/*	length = sprintf(msg_buf, "AT+RELI1");
    send_data(msg_buf, length);
	waitDoingServices(500,0,1);	
*/	length = sprintf(msg_buf,"AT+RESET");
	send_data(msg_buf,length);
	waitDoingServices(5000,0,1);
    //uartDisable();
}


// function to convert a voltage value to a battery percentage
uint8 batteryPercent(uint16 val){
	XDATA float pct = val;
	if(send_debug)
		printf_fast_f("batteryPercent val: %f\r\n", pct);
	// if val is >100 (ADC 0.217V) and < battery_minimum...
	if((val < (settings.battery_minimum - 23)) && (val >settings.battery_minimum - 200)) {
		//save the new minimum value with an offset of approx 1.2% (0.05V)
		settings.battery_minimum = val;
		save_settings = 1;
	}
	// if val is <2047 (ADC maximum 1.2V) and > battery_maximum....
	if((val > (settings.battery_maximum + 23)) && (val < 2047)) {
		//save the new maximum value with an offset of approx 1.2% (0.05V)
		settings.battery_maximum = val;
		save_settings = 1;;
	}
	// otherwise calculate the battery % and return.
	pct = ((pct - settings.battery_minimum)/(settings.battery_maximum - settings.battery_minimum)) * 100;
	//printf_fast("batteryPercent, val:%i, min:%i, max:%i, pct:%i\r\n", val, settings.battery_minimum, settings.battery_maximum, (uint8)pct);
	//printf_fast_f("batteryPercent val:%i, pct:%f\r\n", val,pct);
	if (pct > 100)
		pct = 100.0;
	if (pct < 0)
		pct = 0.0;
	if(send_debug)
		printf_fast_f("batteryPercent returning %f\r\n", pct);
	return (uint8)pct;
}

// structure of a raw record we will send.
typedef struct _RawRecord
{
	uint8	size;	//size of the packet.
	uint8	cmd_code;	// code for this data packet.  Always 00 for a Dexcom data packet.
	uint32	raw;	//"raw" BGL value.
	uint32	filtered;	//"filtered" BGL value 
	uint8	dex_battery;	//battery value
	uint8	my_battery;	//xBridge battery value
	uint32	dex_src_id;		//raw TXID of the Dexcom Transmitter
	//int8	RSSI;	//RSSI level of the transmitter, used to determine if it is in range.
	//uint8	txid;	//ID of this transmission.  Essentially a sequence from 0-63
	uint8	function; // Byte representing the xBridge code funcitonality.  01 = this level.
} RawRecord;

//function to format and send the passed Dexom_packet.
void print_packet(Dexcom_packet* pPkt)
{
	//int dly_time = 0;
	XDATA RawRecord msg;
	//prepare the message
	msg.cmd_code = 0x00;
	msg.raw = dex_num_decoder(pPkt->raw);
	msg.filtered = dex_num_decoder(pPkt->filtered)*2;
	msg.dex_battery = pPkt->battery;
	msg.my_battery = battery_capacity;
	//msg.dex_src_id = dex_tx_id;
	msg.dex_src_id = pPkt->src_addr;
	msg.size = sizeof(msg);
	msg.function = DEXBRIDGE_PROTO_LEVEL; // basic functionality, data packet (with ack), TXID packet, beacon packet (also TXID ack).
	send_data( (uint8 XDATA *)msg, msg.size);
}

//function to print the passed Dexom_packet as either binary or ascii.
/*void printf_packet(Dexcom_packet* pPkt)
{
	char *srcAddr;
	//first normalise the pPkt-txId
	//uint8 txid = (pPkt->txId & 0xFC) >> 2;

	//Next, convert the pPkt->src_addr to a string and store it in the global srcAddr array for printing.
	if(usb_connected) {
		srcAddr = dexcom_src_to_ascii(pPkt->src_addr);
	*/
	/*
	uint8	len;
	uint32	dest_addr;
	uint32	src_addr;
	uint8	port;
	uint8	device_info;
	uint8	txId;
	uint16	raw;
	uint16	filtered;
	uint8	battery;
	uint8	unknown;
	uint8	checksum;
	int8	RSSI;
	uint8	LQI;
	*/
		//print it comma separated.
		//printf_fast("%s,%lu,%lu,%hhu,%hhi,%hhu\r\n",
/*		printf_fast("len:%u, dest_addr:%lu, src_addr:%lu (%s), port:%u, device_info:%u, txId:%u, raw:%u (%lu), filtered:%u (%lu), battery:%u, RSSI:%i, LQI:%u\r\n",
			pPkt->len,
			pPkt->dest_addr,
			pPkt->src_addr,
			srcAddr,
			pPkt->port,
			pPkt->device_info,
			pPkt->txId,
			pPkt->raw,
			dex_num_decoder(pPkt->raw),
			pPkt->filtered,
			2 * dex_num_decoder(pPkt->filtered),
			pPkt->battery,
			pPkt->RSSI,
			pPkt->LQI);
			
			//txid);r	
	}
	
}
*/
//function to send a beacon with the TXID
void sendBeacon()
{
	//char array to store the response in.
	uint8 XDATA cmd_response[7];
	//return if we don't have a connection or if we have already sent a beacon
	if(send_debug)
		printf_fast("%lu: sending beacon Now\r\n", getMs());
	//prepare the response
	//responding with number of bytes,
	cmd_response[0] = sizeof(cmd_response);
	//responding to command 01,
	cmd_response[1] = 0xF1;
	//return the encoded TXID
	memcpy(&cmd_response[2], &settings.dex_tx_id, sizeof(settings.dex_tx_id));
	cmd_response[6] = DEXBRIDGE_PROTO_LEVEL;
	send_data(cmd_response, sizeof(cmd_response));
}


//initialise the USB port
uint8 init_command_buff(t_command_buff* pCmd)
{
	if(!pCmd)
		return 0;
	memset(pCmd->commandBuffer, 0, USB_COMMAND_MAXLEN);
	pCmd->nCurReadPos = 0;
	return 0;
}

//Open the UART and set 9600,n,1 parameters.
void openUart()
{
	XDATA uint8 i=0;
	XDATA uint8 msg[2];
	sprintf(msg,"AT");
    uart1Init();
	uart1SetParity(0);
	uart1SetStopBits(1);
	uartEnable();
	//detect HM-1x baudrate, if not currently set
	if(settings.uart_baudrate >230400) {
		if(send_debug)
			printf_fast("Determining HM-1x baudrate \r\n");
		for(i=0;i<=8;i++) {
			init_command_buff(&command_buff);
			if(send_debug)
				printf_fast("trying %lu\r\n", uart_baudrate[i]);
			settings.uart_baudrate = uart_baudrate[i];
			uart1SetBaudRate(uart_baudrate[i]);
			send_data(msg,sizeof(msg));
			waitDoingServices(500,got_ok,1);
			if(got_ok) break;
		}
		if(!got_ok){
			if(send_debug)
				printf_fast("Could not detect baudrate of HM-1x, setting 9600");
			settings.uart_baudrate=9600;
		}
	}
	uart1SetBaudRate(settings.uart_baudrate); // Set saved baudrate

	init_command_buff(&command_buff);
//	P1DIR |= 0x08; // RTS
	//U1UCR &= ~0x4C;	//disable CTS/RTS on UART1, no Parity, 1 Stop Bit
}


//decode a command received ??
int commandBuffIs(char* command)
{
	uint8 len = strlen(command);
	if(len != command_buff.nCurReadPos)
		return 0;
	return memcmp(command, command_buff.commandBuffer, len)==0;
}

// return code dictates whether to cancel out of current packet wait operation.
// primarily this is so if you change the start channel, it will actually start using that new start channel
// otherwise there's an infinite wait on when it may fail the initial start channel wait, which if it's interfered with may be forever.
//decode and perform the command recieved on a port
int doCommand()
{
	/* format of commands -  
				char 0 = number of bytes in command, including this one.
				char 1 = the command code.
				char [2 to char n]	= the command data.
		Note:  The convention for command responses is to send the command code back ored with 0xF0.  That way the app knows
		which command the response is for.
	*/
	
	/* command 0x01 preceeds a TXID being sent by controlling device on UART1.  Packet length is 6 bytes.
		0x01, lsw2, lsw1, msw2, msw1
	*/
	if(command_buff.commandBuffer[1] == 0x01 && command_buff.commandBuffer[0] == 0x06)
	{
		if(send_debug)
			printf_fast("Processing TXID packet\r\n");
		memcpy(&settings.dex_tx_id, &command_buff.commandBuffer[2],sizeof(settings.dex_tx_id));
		saveSettingsToFlash();
		// send back the TXID we think we got in response
		if(send_debug)
			printf_fast("dex_tx_id: %lu (%s)\r\n", settings.dex_tx_id, dexcom_src_to_ascii(settings.dex_tx_id)); 
		sent_beacon = 0;
		return 0;
	}
	/* command 0xF0 is an acknowledgement sent by the controlling device of a data packet.
		This acknowledgement lets us go to sleep immediately.  Packet length is 2 bytes.
		0x02, 0xF0
	*/
	// if do_sleep is set already, don't process it
	if(command_buff.commandBuffer[0] == 0x02 && command_buff.commandBuffer[1] == 0xF0 && !do_sleep) {
		if(send_debug)
			printf_fast("Processing ACK packet\r\n");
		do_sleep = 1;
		init_command_buff(&command_buff);
		return(0);
	}
	if(command_buff.commandBuffer[0] == 0x53 || command_buff.commandBuffer[0] == 0x73) {
		printf_fast("Processing Status Command\r\nxBridge v%s\r\n", VERSION);
		printf_fast("dex_tx_id: %lu (%s)\r\n", settings.dex_tx_id, dexcom_src_to_ascii(settings.dex_tx_id));
		printf_fast("initialised: %u, sleep_ble: %u, dont_ignore_ble_state: %u, xBridge_hardware: %u, send_debug: %u, do_leds: %u\r\n",
			getFlag(BLE_INITIALISED),
			getFlag(SLEEP_BLE),
			getFlag(DONT_IGNORE_BLE_STATE),
			getFlag(XBRIDGE_HW),
			getFlag(SEND_DEBUG),
			getFlag(DO_LEDS));
		printf_fast("battery_capacity: %u\r\n", battery_capacity);
//		printf_fast("MDMCFG4: %x, MDMCFG3: %x\r\n", MDMCFG4,MDMCFG3); 
//		printf_fast("PKTCTRL1: %x, PKTCTRL0: %x, PKTLEN: %x\r\n", PKTCTRL1, PKTCTRL0, PKTLEN);
		printf_fast("current ms: %lu\r\n", getMs());
	}
	if(command_buff.commandBuffer[0] == 0x44 || command_buff.commandBuffer[0] == 0x64) {
		setFlag(SEND_DEBUG,!getFlag(SEND_DEBUG));
		saveSettingsToFlash();
		send_debug = getFlag(SEND_DEBUG);
		if(send_debug)
			printf_fast("debug output on\r\n");
		else
			printf_fast("debug output off\r\n");
	}
	if(command_buff.commandBuffer[0] == 0x42 || command_buff.commandBuffer[0] == 0x62) {
		setFlag(SLEEP_BLE,!getFlag(SLEEP_BLE));
		saveSettingsToFlash();
		sleep_ble = getFlag(SLEEP_BLE);
		if(sleep_ble)
			printf_fast("BLE Sleeping on\r\n");
		else
			printf_fast("BLE Sleeping off\r\n");
	}
	if(command_buff.commandBuffer[0] == 0x4C || command_buff.commandBuffer[0] == 0x6C) {
		setFlag(DO_LEDS,!getFlag(DO_LEDS));
		saveSettingsToFlash();
		do_leds = getFlag(DO_LEDS);
		if(do_leds)
			printf_fast("LEDs are on\r\n");
		else
			printf_fast("LEDs are off\r\n");
	}
	if(commandBuffIs("OK")) {
		got_ok = 1;
		return(0);
	}
/*	if(commandBuffIs("OK+SLEE")) {
		ble_sleeping=1;
		return(0);
	}
	if(commandBuffIs("OK+WAKE")) {
		ble_sleeping=0;
		return(0);
	}
*/	// we don't respond to unrecognised commands.
	return 1;
}

// Process any commands on either UART0 or USB COM.
int controlProtocolService()
{
	static uint32 cmd_to;
	// ok this is where we check if there's anything happening incoming on the USB COM port or UART 0 port.
	//trying setting nRet to 0, it should only return 1 if there is something to do.
	int nRet = 1;
	uint8 b;
	//if we have timed out waiting for a command, clear the command buffer and return.
	if(command_buff.nCurReadPos > 0 && (getMs() - cmd_to) > 2000) 
	{
		// clear command buffer if there was anything
		init_command_buff(&command_buff);
		//nRet = 0;
		return nRet;
	}	
	//while we have something in either buffer,
	while((usbComRxAvailable() || uart1RxAvailable()) && command_buff.nCurReadPos < USB_COMMAND_MAXLEN)
	{
	// if it is the USB, get the byte from it, otherwise get it from the UART.
		if (usbComRxAvailable()) {
			b = usbComRxReceiveByte();
		}
		else {
			b = uart1RxReceiveByte();
		}
		//putchar(b);
		if(send_debug)
			printf_fast("%c",b);
		command_buff.commandBuffer[command_buff.nCurReadPos] = b;
		command_buff.nCurReadPos++;
		//printf_fast("%x\r\n", command_buff.commandBuffer[0]);
		// reset the command timeout.
		cmd_to = getMs();
		// if it is the end for the byte string, we need to process the command
		if(command_buff.nCurReadPos == command_buff.commandBuffer[0] || (command_buff.nCurReadPos == 1 && ((command_buff.commandBuffer[0] & 0x5F) == 0x53 )) || (command_buff.nCurReadPos == 1 && ((command_buff.commandBuffer[0] & 0x5F) == 0x44 )) || (command_buff.nCurReadPos == 1 && ((command_buff.commandBuffer[0] & 0x5F) == 0x42 )) || (command_buff.nCurReadPos == 1 && ((command_buff.commandBuffer[0] & 0x5F) == 0x4c )) || (command_buff.nCurReadPos == 2 && command_buff.commandBuffer[0] == 0x4F))
		{
			// ok we got the end of a command;
			if(command_buff.nCurReadPos)
			{
//				printf_fast("Processing Command\r\n");
				// do the command
				nRet = doCommand();
				//re-initialise the command buffer for the next one.
				init_command_buff(&command_buff);
				// break out if we got a breaking command
				if(!nRet)
					return nRet;
			}
		}
		// otherwise, if the command is not up to the maximum length, add the character to the buffer.
	}
	return nRet;
}

// process each of the services we need to be on top of.
// if bWithProtocol is true, also check for commands on both USB and UART
int doServices(uint8 bWithProtocol)
{
	dex_tx_id_set = (settings.dex_tx_id != 0);
	boardService();
	updateLeds();
	usbComService();
	bleConnectMonitor();
/*	if(initialised && ble_connected && !sent_beacon) {
		sent_beacon = 1;
		sendBeacon();
	}
*/
	if(bWithProtocol)
		return controlProtocolService();
	return 1;
}


void swap_channel(uint8 channel, uint8 newFSCTRL0)
{
	do
	{
		RFST = 4;   //SIDLE
	} while (MARCSTATE != 0x01);

	// update this, since offset can change based on channel
	FSCTRL0 = newFSCTRL0;
	CHANNR = channel;
	RFST = 2;   //RX
}

int WaitForPacket(uint32 milliseconds, Dexcom_packet* pkt, uint8 channel)
{
	// store the current wixel milliseconds so we know how long we are waiting.
	uint32 start = getMs();
	// clear the packet pointer we are going to use to detect one.
	uint8 XDATA * packet = 0;
	// set the return code to timeout indication, as it is the most likely outcome.
	int nRet = 0;
	// lastpktid is static, because we need to store it during between calls.
	//set lastpktid to a value of 64, because we should NEVER see this value.
	static uint8 lastpktxid = 64;
	// a variable to use to convert the pkt-txId to something we can use.
	uint8 txid = 0;
	if(send_debug)
		printf_fast("waiting for packet on channel %u for %lu \r\n", channel, milliseconds);
	// safety first, make sure the channel is valid, and return with error if not.
	if(channel >= NUM_CHANNELS)
		return -1;
	// set the channel parameters using swap_channel
	swap_channel(nChannels[channel], fOffset[channel]);
	// while we haven't reached the delay......
	while (!milliseconds || (getMs() - start) < milliseconds)
	{
		//see if anything is required to be done immediately, like process a command on the USB.
		//printf_fast("start: %lu, getMs: %lu\r", start, getMs());
		if(!doServices(1) || (getMs() - start) > 320000)
			return -1;			// cancel wait, and cancel calling function
	
		if (packet = radioQueueRxCurrentPacket())
		{
			uint8 len = packet[0];
			// if the packet passed CRC
			if(radioCrcPassed())
			{
				// there's a packet!
				// Add the Frequency Offset Estimate from the FREQEST register to the channel offset.
				// This helps keep the receiver on track for any drift in the transmitter.
				fOffset[channel] += FREQEST;
				// fetch the packet.
				// length +2 because we append RSSI and LQI to packet buffer, which isn't shown in len
				memcpy(pkt, packet, min8(len+2, sizeof(Dexcom_packet))); 
				
				// is the pkt->src_addr the one we want?
				if(pkt->src_addr == settings.dex_tx_id || settings.dex_tx_id == 0)
				{
					// subtract channel index from transaction ID.  This normalises it so the same transaction id is for all transmissions of a same packet
					// and makes masking the last 2 bits safe regardless of which channel the packet was acquired on
					pkt->txId -= channel;
					//convert pkt->txId to something we understand
					txid = (pkt->txId & 0xFC) >> 2;
					// Is this packet different from the one we got last time?
					// This test prevents us getting the same packet on multiple channels.
					if(txid != lastpktxid) 
					{
						// ok, packet is valid, and we haven't seen it before.
						// set the return code and save the packet id for later.
						nRet = 1;
						// save this packet txid for next time.
						lastpktxid = txid;
					}
					last_channel = channel;
				}
			}
			else {
				if(send_debug)
					printf_fast("bad CRC on channel %d\r\n", channel);
				nRet = -2;
			}
			// the line below can be commented/uncommented for debugging.
			//else 
			// pull the packet off the queue, so it isn't there next time we look.
			radioQueueRxDoneWithPacket();
			//return the correct code.
			return nRet;
		}
	}
	// we timed out waiting for the packet.
	//printf_fast("timed out waiting for packet                                                \r\n");
	return nRet;
}

/*get_packet - 	A function to scan the 4 Dexcom channels.  Waits on channel 0 for ~300s from last packet,
				then waits on channels 1, 2, & 3 for 500ms each.
	Parameters:
		pPkt	-	Pointer to a Dexcom_packet.
		
	Uses:
		pkt_time 
				Sets this to the time the packet was aquired.
		WaitForPacket(milliseconds, pkt, channel)
				This function is called to obtain packets.  The return value determines if a packet is
				available to process, or if something else occurred.
				get_packet uses a fixed delay of 25 ms on each channel. This is more than adequate to 
				recieve a packet, which is about 4ms in length.  It cannot be much lower, due to the wixel
				processing speed.  If you go too low, it will cause the wixel to ignore input on the
				USB.  Since we need to set the Transmitter ID, this is not a good thing.
				The Dexcom Tranmsitter sends out a packet in each channel, roughly 500ms apart, every
				5 minutes.  The combination of get_packet and WaitForPacket ensures we see at least
				one of the 4 packets each 5 minutes.  WaitForPacket includes code to exclude duplicate
				packets in each cycle.  We grab the first packet with an ID we didn't see last time, and
				ignore any packets that appear with the same ID.
	Returns:
		0		- 	No Packet received, or a command recieved on the USB.
		1		-	Packet recieved that passed CRC.
*/
int get_packet(Dexcom_packet* pPkt)
{
	static BIT timed_out = 0;
	static BIT crc_error = 0;
	//static uint32 last_cycle_time;
	//uint32 now=0;
	uint32 delay = 0;
	//variable holding an index to each channel parameter set.  Set to the first channel.
	int nChannel = 0;
	//printf_fast("getting Packet\r");
	// start channel is the channel we initially do our infinite wait on.
	for(nChannel = START_CHANNEL; nChannel < NUM_CHANNELS; nChannel++)
	{
		// if we are not on the start channel, we are delaying only for 500ms to capture a packet on the channel.
		if(nChannel != START_CHANNEL) 
		{
			delay=500;
		}
		// else, if we are on the start channel and we timed out, we are going to delay for 298 seconds (300 - 2 seconds for channels 1-3)
		else if (timed_out && (nChannel == 0)) 
		{
			delay=298000;
		}
		// otherwise.....
		else
		{
			// if we haven't captured a packet (pkt_time =0), we sit on channel 0 until we do, so set the delay to 0
			if (!pkt_time) {
				delay = 0;
			}
			// if we got a packet previously (pkt_time != 0), we calculate when we are expecting a packet.
			else if(getMs()<pkt_time) 
			{
				//the current time is less than the pkt_time, meaning our ms register has rolled over.
				delay = (300000 + wake_before_packet) - (4294967295 + getMs() - pkt_time);
			}
			else
			{
				// the current time is greater than the last pkt_time, so we just do a basic calculation.
				// 5 mins in ms + the wake_before_packet time - (current ms - last packet ms)
				delay = (300000 + wake_before_packet) - (getMs() - pkt_time);
			}
			//if we have a delay number that doesn't equal 0, we need to subtract 500 * the channel we last captured on.
			// ie, if we last captured on channel 0, subtract 0.  If on channel 1, subtract 500, etc
			if(delay)
			{
				delay -= (last_channel * 500);
				if(crc_error) 
					delay += 50;
				if(send_debug)
					printf_fast("%lu: last_channel is %u, delay is %lu\r\n", getMs(), last_channel, delay);
			}
			// in case the figure we came up with is greater than 5 minutes, we deal with it here.  Probably never will run, i'm just like that.
			while(delay > 300000)
			{
				delay -= 300000;
			}
		}
		switch(WaitForPacket(delay, pPkt, nChannel))
		{
			case 1:			
				// got a packet that passed CRC
					pkt_time = getMs();
					timed_out = 0;
					if(send_debug)
						printf_fast("got a packet at %lu on channel %u\r\n", pkt_time, last_channel);
					return 1;
			case 0:
				// timed out
				//printf_fast("timed out\r");
				//last_cycle_time=now;
				timed_out = 1;
				continue;
			case -1:
			{
			// cancelled by inbound data on USB (command), or channel invalid.
				//printf_fast("leaving getPacket\r\n");
				return 0;
			}
			case -2:
			{
				//got a bad CRC on the channel, so say so to let the delay calculation know to wait a little longer.
				crc_error = 1;
			}
		}
		//delay = 600;
	}
	//printf_fast("leaving getPacket\r\n");
	return 0;
}

// you can uncomment this if you want a glowing yellow LED when a terminal program is connected
// to the USB.  I got sick of it.
// LineStateChangeCallback - sets the yellow LED to the state of DTR on the USB, whenever it changes.
void LineStateChangeCallback(uint8 state)
{
	//LED_YELLOW(state & ACM_CONTROL_LINE_DTR);
	usb_connected = state & ACM_CONTROL_LINE_DTR;
}

//extern void basicUsbInit();

void main()
{   
	uint8 i = 0;
	uint16 rpt_pkt=0;
	XDATA uint32 tmp_ms = 0;
	systemInit();
	//initialise the USB port
	usbInit();
	//initialise the dma channel for working with flash.
	dma_Init();
	//initialise sleep library
	sleepInit();
	//initialise the radio_mac library
	radioQueueInit();
	// initialise the Radio Regisers
	dex_RadioSettings();
	MCSM0 &= 0x34;			// calibrate every fourth transition to and from IDLE.
	MCSM1 = 0x00;			// after RX go to idle, we don't transmit
	//MCSM2 = 0x08;
	MCSM2 = 0x17;			// terminate receiving on drop of carrier, but keep it up when packet quality is good.
	// implement the USB line State Changed callback function.  
	usbComRequestLineStateChangeNotification(LineStateChangeCallback);
	//configure the P1_2 and P1_3 IO pins
	//makeAllOutputs(LOW);
	setPort1PullType(LOW);
	setDigitalInput(12,PULLED);
	//initialise Anlogue Input 0
	P0INP = 0x1;
	//delay for 30 seconds to get putty up.
	waitDoingServices(10000,0,1);
	printf_fast("Starting xBridge v%s\r\nRetrieving Settings\r\n", VERSION);
	memcpy(&settings, (__xdata *)FLASH_SETTINGS, sizeof(settings));
	//detect if we have xBridge or classic hardware
	
	//turn on HM-1x using P1_0
	setDigitalOutput(10,HIGH);
	//if P1_2 goes high in 1s, it is xBridge
	tmp_ms = getMs();
	while (getMs() < tmp_ms + 1000) {	
//		waitDoingServices(50,0,1);
//		printf_fast("P1_2 is %u\r\n",P1_2);
		if(P1_2) break;
	}
	if(!P1_2) {
		setFlag(XBRIDGE_HW,0);
	}
	//wait 1 seconds, just in case it needs to settle.
	waitDoingServices(1000,0,0);
	//initialise the command buffer
	init_command_buff(&command_buff);
	// Open the UART and set it up for comms to HM-10
	openUart();
	//configure the bluetooth module
	//if we haven't written a 0 into the appropriate flag...
	if(getFlag(BLE_INITIALISED)) { 
		//configure the BT module
		configBt();
		// set the flag to 0
		setFlag(BLE_INITIALISED, 0);
		//set up the value for writing to flash
		save_settings = 1;
	}
	//printf_fast("battery_minimum: %u, battery_maximum: %u\r\n", battery_minimum, battery_maximum);
	//if they are 0xFFFF, then they haven't been stored in flash, set them to something reasonable.
	if(settings.battery_minimum == 0xFFFF || settings.battery_maximum == 0xFFFF) {
		if(getFlag(XBRIDGE_HW)) {
			printf_fast("xBridge hardware circuit selected\r\n");
			setFlag(SLEEP_BLE, 1);
			setFlag(DONT_IGNORE_BLE_STATE, 1);
			settings.battery_maximum = BATTERY_MAXIMUM;
			settings.battery_minimum = BATTERY_MINIMUM;
		} else {
			printf_fast("xDrip-wixel hardware circuit selected\r\n");
			setFlag(SLEEP_BLE, 0);
			setFlag(DONT_IGNORE_BLE_STATE, 0);
			settings.battery_maximum = BATTERY_MAXIMUM_CLASSIC;
			settings.battery_minimum = BATTERY_MINIMUM_CLASSIC;
		}
		save_settings = 1;
		sleep_ble = getFlag(SLEEP_BLE);
		send_debug = getFlag(SEND_DEBUG);
	}
	// measure the initial battery capacity.
	battery_capacity = batteryPercent(adcRead(0 | ADC_REFERENCE_INTERNAL));
	//printf_fast("%lu - battery_capacity: %u\r\n", battery_capacity);
	// we haven't sent a beacon packet yet, so say so.
	sent_beacon = 0;
	LED_GREEN(1);
	last_beacon = getMs();
	// read the flash stored value of our TXID.
	// we do this by reading the address we are interested in directly, cast as a pointer to the 
	// correct data type.
	if(settings.dex_tx_id >= 0xFFFFFFFF) 
		settings.dex_tx_id = 0;
	// store the time we woke up.
	// if dex_tx_id is zero, we do not have an ID to filter on.  So, we keep sending a beacon every 5 seconds until it is set.
	// Comment out this while loop if you wish to use promiscuous mode and receive all Dexcom tx packets from any source (inadvisable).
	// Promiscuous mode is allowed in waitForPacket() function (dex_tx_id == 0, will match any dexcom packet).  Just don't send the 
	// wixel a TXID packet.
	initialised = 1;
	while(settings.dex_tx_id == 0) {
		if(send_debug)
			printf_fast("No dex_tx_id.  Sending beacon.\r\n");
		// wait until we have a BLE connection
		while(!ble_connected) doServices(1);
		//send a beacon packet
		sendBeacon();
		doServices(0);
		//wait 5 seconds
		waitDoingServices(10000, dex_tx_id_set, 1);
	}
	// if we still have settings to save (no TXID set), save them
	if(save_settings)
		saveSettingsToFlash();
	// retrieve the show_leds and send_debug settings
	sleep_ble = getFlag(SLEEP_BLE);
	do_leds = getFlag(DO_LEDS);
	send_debug = getFlag(SEND_DEBUG);
	//radioCalibration();
	// store the last beacon time
	//printf_fast("\r\n");
	// MAIN LOOP
	// initialise the Radio Regisers
	setRadioRegistersInitFunc(dex_RadioSettings);
	if(send_debug)
		printf_fast("looking for %lu (%s)\r\n",settings.dex_tx_id, dexcom_src_to_ascii(settings.dex_tx_id));
	while (1)
	{
		Dexcom_packet Pkt;
//		LED_RED(0);
//		LED_GREEN(1);
		
		if(get_packet(&Pkt) == 0) {
			//printf_fast("last_beacon: %lu, getMs(): %lu\r", last_beacon, getMs());
			if(ble_connected) 
//				printf_fast("\r\nSending Beacon\r\n");
				sendBeacon();
			continue;
		}

		LED_GREEN(0);
		// ok, we got a packet
		// when we send a packet, we wait until we get an ACK to put us to sleep.
		// we only wait a maximum of two minutes
		LED_RED(0);
		if(send_debug)
			printf_fast("%lu - got pkt\r\n", getMs());
		while (!do_sleep){
			while(!ble_connected && (getMs() - pkt_time)<120000) {
				if(send_debug)
					printf_fast("%lu - packet waiting\r\n", getMs());
				setDigitalOutput(10,HIGH);
				waitDoingServices(1000, ble_connected,0);
			}
			if(ble_connected) {
				//printf_fast("%lu - ble_connected: %u, sent_beacon: %u\r\n", getMs(), ble_connected, sent_beacon);
				if(send_debug)
					printf_fast("%lu - send pkt\r\n", getMs());
				// send the data packet
				print_packet(&Pkt);
			} else {
				if(send_debug)
					printf_fast("%lu - ble connect didn't occur, sleeping\r\n", getMs());
				do_sleep = 1;
			}
			//save settings to flash if we need to
			if(save_settings)
				saveSettingsToFlash();
			// wait 10 seconds, listenting for the ACK.
			if(send_debug)
				printf_fast("%lu - waiting for ack\r\n", getMs());
			waitDoingServices(10000, 0, 1);
			
			// if we got the ACK, get out of the loop.
			// if we have sent a number of packets and still have not got an ACK, time to sleep.  We keep trying for up to 3 minutes.
			if((getMs() - pkt_time) >= 120000) {
				//sendBeacon();
				sent_beacon = 0;
				do_sleep = 1;
			}
		}
			
		// can't safely sleep if we didn't get an ACK, or if we are already sleeping!
		if (do_sleep && !is_sleeping)
		{
			// save all Port Interrupts state (enabled/disabled).
		    uint8 savedPICTL = PICTL;
			// save port 0 Interrupt Enable state.  This is a BIT value and equates to IEN1.POIE.
			// This is probably redundant now, as we do all IEN registers in goToSleep.
			BIT savedP0IE = P0IE;
			uint8 savedP0SEL = P0SEL;
			uint8 savedP0DIR = P0DIR;
			uint8 savedP1SEL = P1SEL;
			uint8 savedP1DIR = P1DIR;
			//printf_fast("%lu - going to sleep\r\n", getMs());
			// clear sent_beacon so we send it next time we wake up.
			sent_beacon = 0;
			// turn of the RF Frequency Synthesiser.
			RFST = 4;   //SIDLE
			// turn all wixel LEDs on
			//LED_RED(1);
			//LED_YELLOW(1);
			//LED_GREEN(1);
			// wait 500 ms, processing services.
			dly_ms=getMs();
			while((getMs() - dly_ms) <= 500) {
				// allow the wixel to complete any other tasks.
				doServices(1);
				// if we are writing flash rignt now, reset the delay to wait again.
				if(writing_flash)
					dly_ms=getMs();
			}
			// make sure the uart send buffer is empty.
			while(uart1TxAvailable() < 255);
			// turn the wixel LEDS off
			LED_RED(0);
			LED_YELLOW(0);
			LED_GREEN(0);
			// turn off the BLE module
			if(sleep_ble){
				if(send_debug) {
					printf_fast("turning off BLE\r\n");
					}
				setDigitalOutput(10,LOW);
				ble_connected = 0;
			}
			// sleep for around 300s
			if(send_debug)
				printf_fast("%lu - sleeping for %u\r\n", getMs(), 300-wake_wixel_before_packet);
			radioMacSleep();
			goToSleep(300-wake_wixel_before_packet);   //
			got_packet = 0;
			radioMacResume();
			// reset the UART
			openUart();
			//WDCTL=0x0B;
			// still trying to find out what this is about, but I believe it is restoring state.
			// restore all Port Interrupts we had prior to going to sleep. 
			PICTL = savedPICTL;
			// restore Port 0 Interrupt Enable state.  This is a BIT that equates to IEN1.P0IE.
			// This is probably redundant now, as we already do this in ISR ST.
			P0IE = savedP0IE;
			P0SEL = savedP0SEL;
			P0DIR = savedP0DIR;
			P1SEL = savedP1SEL;
			P1DIR = savedP1DIR;
			// Enable suspend detection and disable any other weird features.
			USBPOW = 1;
			// Without this, we USBCIF.SUSPENDIF will not get set (the datasheet is incomplete).
			USBCIE = 0b0111;
			LED_GREEN(1);
			if(send_debug)
				printf_fast("%lu - awake!\r\n", getMs());
			// get the most recent battery capacity
			battery_capacity = batteryPercent(adcRead(0 | ADC_REFERENCE_INTERNAL));
			//printf_fast("%lu - battery_capacity: %u\r\n", battery_capacity);
			init_command_buff(&command_buff);
			// tell the radio to remain IDLE when the next packet is recieved.
			MCSM1 = 0;			// after RX go to idle, we don't transmit
			// watchdog mode??? this will do a reset?
			//			WDCTL=0x0B;
			// delayMs(50);    //wait for reset
			// clear do_sleep, cause we have just woken up.
			do_sleep = 0;
			// power on the BLE module
			ble_connected = 0;
			//printf_fast("%lu - ble on\r\n", getMs());
			setDigitalOutput(10,HIGH);
			waitDoingServices(250,0,1);
		}
	}
}
