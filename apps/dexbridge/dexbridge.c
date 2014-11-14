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
//#include "dexbridge.h"
#include <wixel.h>
#include <usb.h>
#include <usb_com.h>

#include <radio_registers.h>
#include <radio_queue.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <uart0.h>
#include <gpio.h>

static volatile BIT do_sleep = 0;
static volatile BIT is_sleeping = 0;
//static volatile BIT do_verbose = 0;
//static volatile BIT do_binary = 0;
static volatile int start_channel = 0;
static volatile BIT do_close_usb = 1;
static volatile long dex_tx_addr = 0;
char srcAddr[6];

// forward prototypes
int doServices(uint8 bWithProtocol);
// prototype for get_src_value function
//
// prototype for ascii_to_dexcom_src function
uint32 asciiToDexcomSrc(char *addr);
uint32 getSrcValue(char srcVal);


#define USB_COMMAND_MAXLEN	(32)
#define NUM_CHANNELS		(4)

// frequency offsets for each channel - seed to 0.
static uint8 fOffset[NUM_CHANNELS] = {0xCE,0xD5,0xE6,0xE5};
static uint8 nChannels[NUM_CHANNELS] = { 0, 100, 199, 209 };


// store RF config in FLASH, and retrieve it from here to put it in proper location (also incidentally in flash).
// this allows persistent storage of RF params that will survive a restart of the wixel (although not a reload of wixel app obviously).
// TO-DO get this working with DMA - need to erase memory block first, then write this to it.
uint8 XDATA RF_Params[50];


void LoadRFParam(unsigned char XDATA* addr, uint8 default_val)
{
		*addr = default_val; 
}


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
	
	RF_Params[49] = 1;
}

// Dexcom Transmitter Source Address to match against.
uint32 dex_tx_id; 

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

//getPacketPassedChecksum - returns the checksum of the Dexcom_packet passed to it.
uint8 getPacketPassedChecksum(Dexcom_packet* p)
{
	//take the LQI value from the packet, AND with 0x80.  If it equals 0x80, then return 1, otherwise return 0.
	return ((p->LQI & 0x80)==0x80) ? 1:0;
}

//Open the USB UART, and set 9600,n,1 parameters.
void openUart()
{
    uart0Init();
    uart0SetBaudRate(9600);
    uart0SetParity(0);
    uart0SetStopBits(1);
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
    // The wixel docs note that any input pins consume ~30uA
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

//	LED_YELLOW(radioQueueRxCurrentPacket());
//	LED_RED(0);
}

// This is called by printf and printPacket.
void putchar(char c)
{
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

//format an array to decode the dexcom transmitter name from a Dexcom packet source address.
char SrcNameTable[32] = { '0', '1', '2', '3', '4', '5', '6', '7',
						  '8', '9', 'A', 'B', 'C', 'D', 'E', 'F',
						  'G', 'H', 'J', 'K', 'L', 'M', 'N', 'P',
						  'Q', 'R', 'S', 'T', 'U', 'W', 'X', 'Y' };

// convert the passed uint32 Dexcom source address into an ascii string in the passed char addr[6] array.
void dexcom_src_to_ascii(uint32 src, char addr[6])
{
	//each src value is 5 bits long, and is converted in this way.
	addr[0] = SrcNameTable[(src >> 20) & 0x1F];		//the last character is the src, shifted right 20 places, ANDED with 0x1F
	addr[1] = SrcNameTable[(src >> 15) & 0x1F];		//etc
	addr[2] = SrcNameTable[(src >> 10) & 0x1F];		//etc
	addr[3] = SrcNameTable[(src >> 5) & 0x1F];		//etc
	addr[4] = SrcNameTable[(src >> 0) & 0x1F];		//etc
	addr[5] = 0;	//end the string with a null character.
}


//convert ascii passed to unint32 Dexcom source address.

uint32 asciiToDexcomSrc(char addr[6])
{
	uint32 src = 0;
//	unsigned char i = 0;
	src |= (getSrcValue(addr[0]) << 20);
	src |= (getSrcValue(addr[1]) << 15);
	src |= (getSrcValue(addr[2]) << 10);
	src |= (getSrcValue(addr[3]) << 5);
	src |= getSrcValue(addr[4]);
	//printf("asciiToDexcomSrc: val=%u, src=%u\r\n", val, src);
	return src;
}

//find the value of the src addr character.  Called by ascii_to_dexcom_src
uint32 getSrcValue(char srcVal)
{
	uint8 i = 0;
	for(i = 0; i < 32; i++)
	{
			if (SrcNameTable[i]==srcVal) break;
	}
	//printf("getSrcVal: %c %u\r\n",srcVal, i);
	return i & 0xFF;
}


// structure of a raw record from the dex.
/*typedef struct _RawRecord
{
	uint8	size;	//size of the packet.
	uint32	tickcount;	//tick count it was received
	uint8	src_addr[6];	//src_addr of the device that sent it
	uint16	raw;	//"raw" BGL value.
	uint16	filtered;	//"filtered" BGL value 
	uint8	battery;	//battery value
	int8	RSSI;	//RSSI level of the transmitter, used to determine if it is in range.
	uint8	txid;	//ID of this transmission.  Essentially a sequence from 0-63
} RawRecord;
*/

//function to print the passed Dexom_packet as either binary or ascii.
void print_packet(Dexcom_packet* pPkt)
{
	uint8 txid = (pPkt->txId & 0xFC) >> 2;

	//char srcAddr[6];
	dexcom_src_to_ascii(pPkt->src_addr, srcAddr);
	//dex_tx_id = asciiToDexcomSrc(dex_tx_id_str);
	//printf("myDexID is %s, %x\r\n", dex_tx_id_str, dex_tx_id);
	printf("%s, %lu, ", srcAddr, pPkt->src_addr);
	printf("%lu, %lu, %hhu, %hhi, %hhu\r\n", 
		dex_num_decoder(pPkt->raw),
		2 * dex_num_decoder(pPkt->filtered),
		pPkt->battery,
		getPacketRSSI(pPkt),
		txid);
}

//structure of a USB command string
typedef struct _usb_command
{
	uint8 usbCommandBuffer[USB_COMMAND_MAXLEN];
	uint8 nCurReadPos;
} t_usb_command;

//initialise the USB port
uint8 init_usb_command(t_usb_command* pCmd)
{
	if(!pCmd)
		return 0;
	memset(pCmd->usbCommandBuffer, 0, USB_COMMAND_MAXLEN);
	pCmd->nCurReadPos = 0;
	return 0;
}

//create a static command structure to use.
static t_usb_command usb_command;

//decode a command received ??
int usb_command_is(char* command)
{
	uint8 len = strlen(command);
	if(len != usb_command.nCurReadPos)
		return 0;
	return memcmp(command, usb_command.usbCommandBuffer, len)==0;
}

//convert a string character to an unsigned int byte
uint8 Hex1ToUint4(char c)
{
	if(c >= '0' && c <= '9')
		return c-'0';
	if(c >= 'A' && c <= 'F')
		return c + 10 -'A';
	if(c >= 'a' && c <= 'f')
		return c + 10 -'a';
	return 0;
}

// return code dictates whether to cancel out of current packet wait operation.
// primarily this is so if you change the start channel, it will actually start using that new start channel
// otherwise there's an infinite wait on when it may fail the initial start channel wait, which if it's interfered with may be forever.

//decode and perform the command recieved on the USB port
int doUsbCommand()
{
	if(usb_command_is("HELLO"))
	{
		printf("OK WIXEL Dexbridge 0.1\r\n");
		printf("OK current tick %lu\r\n", getMs());
		printf("OK sleep mode is %s\r\n", (do_sleep)?"ON":"OFF");
		return 1;
	}
	if(usb_command_is("SLEEP ON"))
	{
		do_sleep = 1;
		printf("OK sleep mode on\r\n");
		return 1;
	}
	if(usb_command_is("SLEEP OFF"))
	{
		do_sleep = 0;
		printf("OK sleep mode off\r\n");
		return 1;
	}
	if(memcmp(usb_command.usbCommandBuffer, "TXID", 4) == 0)
	{
		if(usb_command.usbCommandBuffer[4] == ' ') 
		{
			//we are being given a TX ID integer (already encoded)
			//printf("%s\r\n",&usb_command.usbCommandBuffer[5]);
			//dex_tx_id = atol(&usb_command.usbCommandBuffer[5]);
			dex_tx_id = asciiToDexcomSrc(&usb_command.usbCommandBuffer[5]);
		}
		dexcom_src_to_ascii(dex_tx_id, srcAddr);
		//printf("OK TXID is %lu\r\n", dex_tx_id);
		printf("OK TXID is %s\r\n", srcAddr);
		return 0;
	}
	printf("Unrecognised command\r\n");
	return 1;
}


int usbControlProtocolService()
{
	// ok this is where we check if there's anything happening incoming on the COM port.
	int nRet = 1;
	while(usbComRxAvailable())
	{
		uint8 b = usbComRxReceiveByte();

		if(b == '\r' || b == '\n')
		{
			printf("\r\n");
			// ok we got the end of a command;
			if(usb_command.nCurReadPos)
			{
				// do the command
				nRet = doUsbCommand();
				init_usb_command(&usb_command);
				// break out if we got a breaking command
				if(!nRet)
					return nRet;
			}
		}
		else if(usb_command.nCurReadPos < USB_COMMAND_MAXLEN)
		{
			usb_command.usbCommandBuffer[usb_command.nCurReadPos++]=b;
		}
		else
		{
			// clear command if there was anything
			init_usb_command(&usb_command);
			printf("INVALID COMMAND\r\n");
		}
	}
	return nRet;
}

int doServices(uint8 bWithProtocol)
{
	boardService();
	updateLeds();
	usbComService();
	if(bWithProtocol)
		return usbControlProtocolService();
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

// channel is the channel index = 0...3
int WaitForPacket(uint16 milliseconds, Dexcom_packet* pkt, uint8 channel)
{
	uint32 start = getMs();
	uint8 XDATA * packet = 0;
	int nRet = 0;
	
	// safety first
	if(channel >= NUM_CHANNELS)
		return -1;
	
	swap_channel(nChannels[channel], fOffset[channel]);

/*	if(do_verbose)
		printf("[%lu] starting wait for packet on channel %d(%d) - will wait for %u ms\r\n", start, channel, (int)CHANNR, milliseconds);
*/
	while (!milliseconds || (getMs() - start) < milliseconds)
	{
		if(!doServices(1))
			return -1;			// cancel wait, and cancel calling function
	
		if (packet = radioQueueRxCurrentPacket())
		{
			uint8 len = packet[0];

			if(radioCrcPassed())
			{
				fOffset[channel] += FREQEST;
				// there's a packet!
				memcpy(pkt, packet, min8(len+2, sizeof(Dexcom_packet))); // +2 because we append RSSI and LQI to packet buffer, which isn't shown in len
/*				if(do_verbose)
					printf("[%lu] received packet channel %d(%d) RSSI %d offset %02X bytes %hhu\r\n", getMs(), channel, (int)CHANNR, getPacketRSSI(pkt), fOffset[channel], len);
*/
					nRet = 1;
				// subtract channel index from transaction ID.  This normalises it so the same transaction id is for all transmissions of a same packet
				// and makes masking the last 2 bits safe regardless of which channel the packet was acquired on
				pkt->txId -= channel;
			}
/*			else
			{
				if(do_verbose)
					printf("[%lu] CRC failure channel %d(%d) RSSI %d %hhu bytes received\r\n", getMs(), channel, (int)CHANNR, (int)((int8)(RSSI))/2 - 73, len);
			}
*/
			// pull the packet off the queue
			radioQueueRxDoneWithPacket();
			return nRet;
		}
	}

/*	if(do_verbose)
		printf("[%lu] timed out waiting for packet on channel %d(%d)\r\n", getMs(), channel, (int)CHANNR);
*/	
	return nRet;
}

int get_packet(Dexcom_packet* pPkt)
{
	int delay = 0;								// initial delay is infinite (unless receive cancelled by protocol on USB)
	int nChannel = 0;
	
	// start channel is the channel we initially do our infinite wait on.
	for(nChannel = start_channel; nChannel < NUM_CHANNELS; nChannel++)
	{
		// initial receive packet call blocks forever. 
		switch(WaitForPacket(delay, pPkt, nChannel))
		{
			case 1:			// got a packet that passed CRC
				if(pPkt->src_addr == dex_tx_id)
					return 1;
				else
					return 0;
			case 0:								// timed out
				break;
			case -1:							// cancelled by protocol on USB (e.g. start channel changed)
			{
//				if(do_verbose)
//					printf("[%lu] wait for packet on channel %d(%d) abandoned\r\n", getMs(), nChannel, (int)CHANNR);
				return 0;
			}
		}
		// ok, no packet this time, set new delay and try next channel
		// set up the delay to wait for 500 ms for a packet.
//		delay = 500;
		delay = 10;
	}
	return 0;
}

void LineStateChangeCallback(uint8 state)
{
	LED_YELLOW(state & ACM_CONTROL_LINE_DTR);
}

extern void basicUsbInit();

void main()
{   
	systemInit();
	usbInit();
	usbComRequestLineStateChangeNotification(LineStateChangeCallback);

	// we actually only use USB, so no point wasting power on UART0
//	openUart();
	init_usb_command(&usb_command);
	
	setRadioRegistersInitFunc(dex_RadioSettings);

	radioQueueInit();
    radioQueueAllowCrcErrors = 1;
	// these are reset in radioQueueInit and radioMacInit after our init func was already called
	MCSM1 = 0;			// after RX go to idle, we don't transmit
	while (1)
	{
		Dexcom_packet Pkt;
		memset(&Pkt, 0, sizeof(Dexcom_packet));

		if(!get_packet(&Pkt))
			continue;

		// ok, we got a packet
		print_packet(&Pkt);
			
		// can't safely sleep if we didn't get a packet!
		if (do_sleep)
		{
		    uint8 savedPICTL = PICTL;
			BIT savedP0IE = P0IE;

			RFST = 4;   //SIDLE
			LED_RED(1);
			LED_YELLOW(1);
			LED_GREEN(1);
			delayMs(80);

			doServices(1);

			LED_RED(0);
			LED_YELLOW(0);
			LED_GREEN(0);

			goToSleep(280);   //~295 s

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


