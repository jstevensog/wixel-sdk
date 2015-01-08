/** DEXBRIDGE:
All thanks to Adrien de Croy, who created dexterity-wixel and modified the
wixel-sdk, upon which dexbridge is based.   

If you are going to build from this source file, please ensure you are using
Adrien's wixel-sdk or it will not compile.

== Description ==
A program to allow a wixel to capture packets from a Dexcom G4 Platinum 
Continuous Glucose Monitor transmitter and send them in text form
out of the USB port.

It also accepts the command TXID, which allows you to see or set the 
Dexcom Transmitter ID in text form (ie 63GEA).  It will NOT display packets
unless TXID is set.  Make sure you set it when the wixel is powered or
detected on USB.

A Wixel running this app appears to the USB host as a Virtual COM Port,
with USB product ID 0x2200.  To view the output of this app, connect to
the Wixel's virtual COM port using a terminal program.  Be sure to set your
terminal's line width to 120 characters or more to avoid line wrapping.
 
The app uses the radio_queue libray to receive packets.  It does not
transmit any packets.

The output from this app takes the following format:

The red LED indicates activity on the radio channel (packets being received).
Since every radio packet has a chance of being lost, there is no guarantee
that this app will pick up all the packets being sent, and some of
what it does pick up will be corrupted (indicated by a failed CRC check).


== Parameters ==

radio_channel: See description in radio_link.h.
*/

/** Dependencies **************************************************************/
#include <wixel.h>
#include <usb.h>
#include <usb_com.h>

#include <radio_registers.h>
#include <radio_queue.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <uart1.h>


//define the FLASH_TX_ID address.  This is the address we store the Dexcom TX ID number in.
#define FLASH_TX_ID		(0x77F8)
//define the maximum command string length for USB commands.
#define USB_COMMAND_MAXLEN	(32)
//defines the number of channels we will scan.
#define NUM_CHANNELS		(4)

static volatile BIT do_sleep = 0;		// indicates we should go to sleep between packets
static volatile BIT is_sleeping = 0;	// flag indicating we are sleeping.
static volatile BIT do_close_usb = 1;	// indicates we should close the USB port when we go to sleep.
static volatile BIT usb_connected;		// indicates DTR set on USB.  Meaning a terminal program is connected via USB for debug.
static volatile BIT sent_beacon;		// indicates if we have sent our current dex_tx_id to the app.
static volatile BIT writing_flash;		// indicates if we are writing to flash.
static volatile int start_channel = 0;	// the radio channel we will start looking for packets on.
// char array to use to convert the Dexcom source address long to a string.
char srcAddr[6];
// Dexcom Transmitter Source Address to match against.
uint32 dex_tx_id; 

// forward prototypes
// prototype for doServices function.
int doServices(uint8 bWithProtocol);
// prototype for getSrcValue function
uint32 getSrcValue(char srcVal);



// frequency offsets for each channel - seed to 0.
static uint8 fOffset[NUM_CHANNELS] = {0xCE,0xD5,0xE6,0xE5};
static uint8 nChannels[NUM_CHANNELS] = { 0, 100, 199, 209 };


// writeBuffer holds the data from the computer that we want to write in to flash.
XDATA uint32 writeBuffer;

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
	// first erase the page
	FADDR = address >> 1;	// not sure if i need to do this again.
	flashWriteDmaConfig.VLEN_LENH = length >> 8;
	flashWriteDmaConfig.LENL = length;
	DMAIRQ &= ~(1<<0);		// Clear DMAIF0 so we can poll it to see when the transfer finishes.
	DMAARM |= (1<<0);
	__asm lcall _startFlashWrite __endasm;
	while(!(DMAIRQ & (1<<0))){}	// wait for the transfer to finish by polling DMAIF0
	while(FCTL & 0xC0){}		// wait for last word to finish writing by polling BUSY and SWBUSY
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
void uartEnable() {
    U1UCR &= ~0x40; //CTS/RTS Off.  We always want it off.
    U1CSR |= 0x40; // Recevier disable
}

void uartDisable() {
    U1UCR &= ~0x40; //CTS/RTS Off. We always want it off.
    U1CSR &= ~0x40; // Recevier disable.
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
	// in our case = 24000000 * (256+17) x 2 ^ (-17) = (24000000 / 131072) * 273 = 49987.79
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
    LoadRFParam(&FOCCFG, 0x0A);		// allow range of +/1 FChan/4 = 375000/4 = 93750.  No CS GATE
    LoadRFParam(&BSCFG, 0x6C);

    // AGC Control:
    // This affects many things, including:
    //    Carrier Sense Absolute Threshold (Sec 13.10.5).
    //    Carrier Sense Relative Threshold (Sec 13.10.6).
    LoadRFParam(&AGCCTRL2, 0x44);
    LoadRFParam(&AGCCTRL1, 0x00);
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
	
	//RF_Params[49] = 1;
}


uint8 min8(uint8 a, uint8 b)
{
	if(a < b) return a;
	return b;
}

// _Dexcom_packet - Type Definition of a Dexcom Radio packet
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

// getPacketRSSI - returns the RSSI value from the Dexcom_packet passed to it.
int8 getPacketRSSI(Dexcom_packet* p)
{
	// RSSI_OFFSET for 489 kbaud is closer to 73 than 71
	return (p->RSSI/2)-73;
}

/*
//getPacketPassedChecksum - returns the checksum of the Dexcom_packet passed to it.
uint8 getPacketPassedChecksum(Dexcom_packet* p)
{
	//take the LQI value from the packet, AND with 0x80.  If it equals 0x80, then return 1, otherwise return 0.
	return ((p->LQI & 0x80)==0x80) ? 1:0;
}
*/
//Open the UART and set 9600,n,1 parameters.
void openUart()
{
    uart1Init();
    uart1SetBaudRate(9600); // Set 9600 baud on UART1
	U1UCR &= ~0x4C;	//disable CTS/RTS on UART1, no Parity, 1 Stop Bit
 //   uart1SetParity(0);
 //   uart1SetStopBits(1);
}

// Configure the BlueTooth module with a name.
void configBt() {
    //uartEnable();
    printf("AT+NAMEDxB%0x%0x%0x%0x\r", serialNumber[3],serialNumber[2],serialNumber[1],serialNumber[0]);
    //uartDisable();
}


/** Functions *****************************************************************/
/* the function that puts the system to sleep (PM2) and configures sleep timer to
wake it again in 250 seconds.*/
void makeAllOutputs(BIT value)
{
    int i = 0;
    for (;i < 16; i++)
	{
		setDigitalOutput(i, value);
    }
}

// use power mode = 1 (PM2)
#define SLEEP_MODE_USING (0x01)

// ISR for catching Sleep Timer interrupts
ISR (ST, 0) 
{
	IRCON &= ~0x80; 	// clear IRCON.STIF
	SLEEP &= ~SLEEP_MODE_USING; 	// clear SLEEP.MODE
	IEN0 &= ~0x20; 		// clear IEN0.STIE
	WORIRQ &= ~0x11; 	// clear Sleep timer EVENT0_MASK and EVENT0_FLAG
	WORCTRL &= ~0x03; 	// Set timer resolution back to 1 period.

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
    // The wixel docs note that any high output pins consume ~30uA
    makeAllOutputs(LOW);

    IEN0 |= 0x20; // Enable global ST interrupt [IEN0.STIE]
    WORIRQ |= 0x10; // enable sleep timer interrupt [EVENT0_MASK]

	/* the sleep mode i've chosen is PM2. According to the CC251132 datasheet,
	typical power consumption from the SoC should be around 0.5uA */
	/*The SLEEP.MODE will be cleared to 00 by HW when power
	mode is entered, thus interrupts are enabled during power modes.
	All interrupts not to be used to wake up from power modes must
	be disabled before setting SLEEP.MODE!=00.*/
    
	// sleep power mode 2 is incompatible with USB - as USB registers lose state in this mode.
	
	SLEEP |= SLEEP_MODE_USING; // SLEEP.MODE = PM2

	if(do_close_usb)
	{
		// disable the USB module
		SLEEP &= ~(1<<7); // Disable the USB module (SLEEP.USB_EN = 0).
		disableUsbPullup();
		usbDeviceState = USB_STATE_DETACHED;
	}
		
    // Reset timer, update EVENT0, and enter PM2
    // WORCTRL[2] = Reset Timer
    // WORCTRL[1:0] = Sleep Timer resolution
    // 00 = 1 period
    // 01 = 2^5 periods
    // 10 = 2^10 periods
    // 11 = 2^15 periods

    // t(event0) = (1/32768)*(WOREVT1 << 8 + WOREVT0) * timer res
    // e.g. WOREVT1=0,WOREVT0=1,res=2^15 ~= 0.9766 second

    WORCTRL |= 0x04; // Reset
    // Wait for 2x+ve edge on 32kHz clock
    temp = WORTIME0;
    while (temp == WORTIME0) {};
    temp = WORTIME0;
    while (temp == WORTIME0) {};

    WORCTRL |= 0x03; // 2^15 periods
    WOREVT1 = (seconds >> 8);
    WOREVT0 = (seconds & 0xff); //300=293 s

    PCON |= 0x01; // PCON.IDLE = 1;
	
	is_sleeping = 1;
}

void updateLeds()
{
	if (do_sleep)
	{
		if(is_sleeping)
		{
			LED_YELLOW((getMs()&0x00000F00) == 0x100);
		}
		else
		{
			LED_GREEN((getMs()&0x00000380) == 0x80);
		}
	}

	LED_RED(radioQueueRxCurrentPacket());
//	LED_RED(0);
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
	for(i=0; i <= len; i++)
	{
		uart1TxSendByte(msg[i]);
	}
	//uart1TxSend((const uint8 XDATA*)&msg, len);
	if(usb_connected) {
		while(usbComTxAvailable() < len) {};
		for(i=0; i <= len; i++)
		{
			usbComTxSendByte(msg[i]);
		}
	//usbComTxSend((uint8 XDATA *)msg, len);
	}
}

// structure of a raw record we will send.
typedef struct _RawRecord
{
	uint8	size;	//size of the packet.
	uint8	cmd_code;	// code for this data packet.  Always 00 for a Dexcom data packet.
	uint32	raw;	//"raw" BGL value.
	uint32	filtered;	//"filtered" BGL value 
	uint8	dex_battery;	//battery value
	uint16	my_battery;	//dexbridge battery value
	uint32	dex_src_id;		//raw TXID of the Dexcom Transmitter
	//int8	RSSI;	//RSSI level of the transmitter, used to determine if it is in range.
	//uint8	txid;	//ID of this transmission.  Essentially a sequence from 0-63
} RawRecord;

//function to print the passed Dexom_packet as either binary or ascii.
void print_packet(Dexcom_packet* pPkt)
{
	XDATA RawRecord msg;
	//prepare the message
	msg.cmd_code = 0x00;
	msg.raw = dex_num_decoder(pPkt->raw);
	msg.filtered = dex_num_decoder(pPkt->filtered)*2;
	msg.dex_battery = pPkt->battery;
	msg.my_battery = adcRead(0 | ADC_BITS_7);
	msg.dex_src_id = dex_tx_id;
	msg.size = sizeof(msg);
	send_data( (uint8 XDATA *)msg, msg.size);
	
}

//function to send a beacon with the TXID
void send_beacon()
{
	//char array to store the response in.
	uint8 XDATA cmd_response[6];
	//prepare the response
	//responding with number of bytes,
	cmd_response[0] = 6;
	//responding to command 01,
	cmd_response[1] = 0xF1;
	//return the encoded TXID
	memcpy(&cmd_response[2], &dex_tx_id, sizeof(dex_tx_id));
	send_data( cmd_response, 6);
}

//structure of a USB command
typedef struct _command_buff
{
	uint8 commandBuffer[USB_COMMAND_MAXLEN];
	uint8 nCurReadPos;
} t_command_buff;


//initialise the USB port
uint8 init_command_buff(t_command_buff* pCmd)
{
	if(!pCmd)
		return 0;
	memset(pCmd->commandBuffer, 0, USB_COMMAND_MAXLEN);
	pCmd->nCurReadPos = 0;
	return 0;
}

//create a static command structure to use.
static t_command_buff command_buff;

//decode a command received ??
int command_buff_is(char* command)
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
	
	/* command 0x01 preceeds a TXID being sent by controlling device on UART1
		0x01, lsw2, lsw1, msw2, msw1
	*/
	if(command_buff.commandBuffer[1] == 0x01 && command_buff.commandBuffer[0] == 0x06)
	{
		//we are being given a TX ID 32 bit integer (already encoded)
		//indicate we are writing to flash to stop any sleeping etc.
		writing_flash=1;
		//printf("%s\r\n",&command_buff.commandBuffer[5]);
		memcpy(&dex_tx_id, &command_buff.commandBuffer[2],sizeof(dex_tx_id));
		//printf("dex_tx_id: %lu\n", dex_tx_id);
		//copy dex_tx_id into the writeBuffer so we can write it to flash
		writeBuffer=dex_tx_id;
		//printf("Erasing....\r\n");
		eraseFlash(FLASH_TX_ID);
		//printf("Writing....\r\n");
		writeToFlash(FLASH_TX_ID, sizeof(dex_tx_id));
		// send back the TXID we think we got in response
		send_beacon();
		writing_flash=0;
		return 0;
	}
	/* command 0xF0 is an acknowledgement sent by the controlling device of a data packet.
		This acknowledgement lets us go to sleep immediately.
		0x02, 0xF0
	*/
	if(command_buff.commandBuffer[0] == 0x02 && command_buff.commandBuffer[1] == 0xF0)
		do_sleep = 1;
	// we don't respond to unrecognised commands.
	return 1;
}

// Process any commands on either UART0 or USB COM.
int controlProtocolService()
{
	static uint32 cmd_to;
	// ok this is where we check if there's anything happening incoming on the USB COM port or UART 0 port.
	int nRet = 1;
	uint8 b;
	//if we have timed out waiting for a command, clear the command buffer and return.
	if(command_buff.nCurReadPos > 0 && (getMs() - cmd_to) > 2000) 
	{
		//printf("TIMEOUT\r\n");
		// clear command buffer if there was anything
		init_command_buff(&command_buff);
		return nRet;
	}	
	//while we have something in either buffer,
	while((usbComRxAvailable() || uart1RxAvailable()) && command_buff.nCurReadPos < USB_COMMAND_MAXLEN)
	{
	// if it is the USB, get the bye from it, otherwise get it from the UART.
		if (usbComRxAvailable()) {
			b = usbComRxReceiveByte();
		}
		else {
			b = uart1RxReceiveByte();
		}
		//putchar(b);
		command_buff.commandBuffer[command_buff.nCurReadPos] = b;
		command_buff.nCurReadPos++;
		// reset the command timeout.
		cmd_to = getMs();
		//printf("byte: %x, pos: %u, ms: %lu\n",b,command_buff.nCurReadPos, cmd_to);
		// if it is the end for the byte string, we need to process the command
		if(command_buff.nCurReadPos == command_buff.commandBuffer[0])
		{
			// ok we got the end of a command;
			if(command_buff.nCurReadPos)
			{
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
	boardService();
	updateLeds();
	usbComService();
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

/* WaitForPacket - Function to wait on a specified channel for a specified period, for a Dexcom packet.
	Parmeters:
		milliseconds	-	The specified number of milliseconds to wait on the channel.
		pkt				-	The Dexcom_packet variable to fill with the packet data.
		channel			-	The channel to wait for the packet on.
	Uses:
		dex_tx_id		-	Global long which is set to the encoded Dexcom Transmitter ID we are 
							looking for a packet from.
		nChannels		- 	Global uint8 Array of channels.
		fOffset			-	Global uint8 Array of frequency offsets.
						The above two arrays specify the radio parameters for the Dexom channels.
		
	Returns:
		0				-	Timed out waiting for the packet.
		1				-	Packet from the desired transmitter was recieved with a valid CRC.
		-1				-	Abandoned waiting for the packet, as a command is available on the USB.
	
*/
int WaitForPacket(uint16 milliseconds, Dexcom_packet* pkt, uint8 channel)
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
	
	// safety first, make sure the channel is valid, and return with error if not.
	if(channel >= NUM_CHANNELS)
		return -1;
	// set the channel parameters using swap_channel
	swap_channel(nChannels[channel], fOffset[channel]);
	// while we haven't reached the delay......
	while (!milliseconds || (getMs() - start) < milliseconds)
	{
		//see if anything is required to be done immediately, like process a command on the USB.
		if(!doServices(1))
			return -1;			// cancel wait, and cancel calling function
	
		if (packet = radioQueueRxCurrentPacket())
		{
			uint8 len = packet[0];
			// if the packet passed CRC
			if(radioCrcPassed())
			{
				// there's a packet!
				// Add the Freqency Offset Estimate from the FREQEST register to the channel offset.
				// This helps keep the receiver on track for any drift in the transmitter.
				fOffset[channel] += FREQEST;
				// fetch the packet.
				// length +2 because we append RSSI and LQI to packet buffer, which isn't shown in len
				memcpy(pkt, packet, min8(len+2, sizeof(Dexcom_packet))); 
				
				// is the pkt->src_addr the one we want?
				if(pkt->src_addr == dex_tx_id || dex_tx_id == 0)
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
						// lines below can be commented/uncommented for debugging
						//printf("channel %d, %d, %d, ", channel, lastpktxid, txid);
						//printf("channel %d, ", channel);
						// save this packet txid for next time.
						lastpktxid = txid;
					}
				}
			}
			// the line below can be commented/uncommented for debugging.
			//else printf("%d bad CRC\r\n", channel);
			// pull the packet off the queue, so it isn't there next time we look.
			radioQueueRxDoneWithPacket();
			//return the correct code.
			return nRet;
		}
	}
	// we timed out waiting for the packet.
	return nRet;
}

/*get_packet - 	A function to scan the 4 Dexcom channels, waiting on each channel for a short period
				to listen for a packet.
	Parameters:
		pPkt	-	Pointer to a Dexcom_packet.
		
	Uses:
				No global variables are used in this function.
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
	static uint32 last_cycle_time;
	uint32 now=0;
	int delay;
	//variable holding an index to each channel parameter set.  Set to the first channel.
	int nChannel = 0;
	last_cycle_time = now;
	
	// start channel is the channel we initially do our infinite wait on.
	for(nChannel = start_channel; nChannel < NUM_CHANNELS; nChannel++)
	{
		now = getMs();
		delay = 50 - (now - last_cycle_time);
		//printf("get_packet last_cycle_time: %u\r", now-last_cycle_time);
		// We only sit on each channel for 25ms.  This is long enough to get a packet (4ms)
		// but not so long as prevent doServices() to process the USB port.
		switch(WaitForPacket(delay, pPkt, nChannel))
		{
			case 1:			
				// got a packet that passed CRC
					return 1;
			case 0:
			// timed out
				continue;
			case -1:
			{
			// cancelled by inbound data on USB (command), or channel invalid.
				return 0;
			}
		}
	}
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
	uint32 dly_ms=0;
	systemInit();
	//initialise Anlogue Input 0
	P0INP = 0x1;
	//initialise the USB port
	usbInit();
	//initialise the dma channel for working with flash.
	dma_Init();
	usbComRequestLineStateChangeNotification(LineStateChangeCallback);
	// we actually only use USB, so no point wasting power on UART
	openUart();
	init_command_buff(&command_buff);
	

	setRadioRegistersInitFunc(dex_RadioSettings);

	radioQueueInit();
    radioQueueAllowCrcErrors = 1;
	// these are reset in radioQueueInit and radioMacInit after our init func was already called
	MCSM1 = 0;			// after RX go to idle, we don't transmit
	sent_beacon = 0;
	while (1)
	{
		Dexcom_packet Pkt;
		memset(&Pkt, 0, sizeof(Dexcom_packet));
		// read the flash stored value of our TXID.
		// we do this by reading the address we are interested in directly, cast as a pointer to the 
		// correct data type.
		dex_tx_id = *(uint32 XDATA *)FLASH_TX_ID;
		if(dex_tx_id >= 0xFFFFFFFF) dex_tx_id = 0;
		//send our current dex_tx_id to the app, to let it know what we are looking for.  Only do this when we wake up (sent_beacon is false).
		if(!sent_beacon)
			send_beacon();
		//continue to loop until we get a packet
		if(!get_packet(&Pkt))
			continue;

		// ok, we got a packet
		print_packet(&Pkt);
		
			
		// can't safely sleep if we didn't get a packet!
		if (do_sleep)
		{
			// not sure what this is about yet, but I believe it is saving state.
		    uint8 savedPICTL = PICTL;
			BIT savedP0IE = P0IE;

			RFST = 4;   //SIDLE
			// clear sent_beacon so we send it next time we wake up.
			sent_beacon = 0;
			// turn all wixel LEDs on
			LED_RED(1);
			LED_YELLOW(1);
			LED_GREEN(1);
			// wait 500 ms, processing services.
			//delayMs(80);
			dly_ms=getMs();
			while((getMs() - dly_ms) < 500) {
				// allow the wixel to complete any other tasks.
				doServices(1);
				// if we are writing flash rignt now, reset the delay to wait again.
				if(writing_flash)
					dly_ms=getMs();
			}
			// turn the wixel LEDS off
			LED_RED(0);
			LED_YELLOW(0);
			LED_GREEN(0);
			// sleep for aroud 300s
			goToSleep(280);   //~295 s
			// still trying to find out what this is about, but I believe it is restoring state.
			PICTL = savedPICTL;
			P0IE = savedP0IE;
			// Enable suspend detection and disable any other weird features.
			USBPOW = 1;
			// Enable the USB common interrupts we care about: Reset, Resume, Suspend.
			// Without this, we USBCIF.SUSPENDIF will not get set (the datasheet is incomplete).
			USBCIE = 0b0111;

			// bootstrap radio again
			radioMacInit();
			MCSM1 = 0;			// after RX go to idle, we don't transmit
			radioMacStrobe();
			
			// watchdog mode??? this will do a reset?
			//			WDCTL=0x0B;
			// delayMs(50);    //wait for reset
		}
	}
}


