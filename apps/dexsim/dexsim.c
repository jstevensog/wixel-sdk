/** DEXSIM:
All thanks to Adrien de Croy, who created dexterity-wixel and modified the
wixel-sdk, upon which dexsim is based.   

If you are going to build from this source file, please ensure you are using
Adrien's wixel-sdk or it will not compile.

== Description ==
A program to simulate Blood Glucose readings from a Dexcom G4 Transmitter


A Wixel running this app appears to the USB host as a Virtual COM Port,
with USB product ID 0x2200.  To view the output of this app, connect to
the Wixel's virtual COM port using a terminal program.  Be sure to set your
terminal's line width to 120 characters or more to avoid line wrapping.
 
The app uses the radio_queue libray to transmit packets.  It does not
recieve any packets.


== Parameters ==

radio_channel: See description in radio_link.h.
*/

/** Dependencies **************************************************************/
#include <wixel.h>
#include <usb.h>
#include <usb_com.h>
#include <radio_com.h>
#include <radio_registers.h>

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <random.h>
#include <math.h>


//define the FLASH_SETTINGS address.  This is the address we store the Dexcom TX ID number, and various other settings for the simulator.
#define FLASH_SETTINGS		(0x77DF)
//define the maximum command string length for USB commands.
#define USB_COMMAND_MAXLEN	(32)
//defines the number of channels we will scan.
#define NUM_CHANNELS		(4)
//defines for calculating mean_raw_bg
#define RAWBG_OFFSET		(32000)
#define RAWBG_FACTOR		(1167)

//storage for the samples for calculation of the next.
volatile XDATA uint32 sample[3];

static volatile BIT usb_connected;		// indicates DTR set on USB.  Meaning a terminal program is connected via USB for debug.
//static volatile int start_channel = 0;	// the radio channel we will start looking for packets on.
// char array to use to convert the Dexcom source address long to a string.
XDATA char srcAddr[6];
// Dexcom Transmitter Source Address to send in packets.
XDATA uint32 dex_tx_id; 

typedef struct _Settings {
	uint32 	dex_tx_id;	//4 bytes
	float	alpha1;		//4 bytes
	float	alpha2;		//4 bytes
	float	sigma_a;	//4 bytes
	uint32	mean_raw_bg;	//4 bytes
} Settings;
XDATA Settings flash_settings;

// forward prototypes
int doServices(uint8 bWithProtocol);
//
// prototype for ascii_to_dexcom_src function
uint32 asciiToDexcomSrc(char *addr);
// prototype for getSrcValue function
uint32 getSrcValue(char srcVal);



// frequency offsets for each channel - seed to 0.
static uint8 fOffset[NUM_CHANNELS] = {0xCE,0xD5,0xE6,0xE5};
static uint8 nChannels[NUM_CHANNELS] = { 0, 100, 199, 209 };


// writeBuffer holds the data that we want to write in to flash.
XDATA uint8 writeBuffer[sizeof(flash_settings)];

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

/* eraseFlash:	A function to erase a page of flash.
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
	// This is being set to 0 dbm
    LoadRFParam(&PA_TABLE0, 0xFE);

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
} Dexcom_packet;


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

void updateLeds()
{
	LED_YELLOW((getMs()&0x00000F00) == 0x100);
	LED_GREEN((getMs()&0x00000380) == 0x80);
	LED_RED(radioComTxAvailable());
}

// This is called by printf and printPacket.
void putchar(char c)
{
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

uint16 dex_num_encoder(uint32 usLongFloat)
{
	uint8 usExponent = 0;
	uint16 usReversed = 0;
	uint32 usMantissa = usLongFloat;
	while (usMantissa < 8191) {
		usMantissa >> 1;
		usExponent++;
	}
	usReversed = usExponent;
	usReversed << 13;
	usReversed = usReversed | usMantissa;
	bit_reverse_bytes((uint8*) &usReversed, 2);
	return usReversed;
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


/* asciiToDexcomSrc - 	function to convert a 5 character string into a unit32 that equals a Dexcom
						transmitter Source address.  The 5 character string is equivalent to the 
						characters printed on the transmitter, and entered into a reciever.
	Parameters:
		addr		-	a 5 character string. eg "63GEA"
	Returns:
		uint32		-	a value equivalent to the incodeded Dexcom Transmitter address.  For use in 
						the WaitForPacket function to filter packets.
	Uses:
		getSrcValue(char)
			This function returns a value equivalent to the character for encoding.
			See srcNameTable[]
*/
uint32 asciiToDexcomSrc(char addr[6])
{
	// prepare a uint32 variable for our return value
	uint32 src = 0;
	// look up the first character, and shift it 20 bits left.
	src |= (getSrcValue(addr[0]) << 20);
	// look up the second character, and shift it 20 bits left.
	src |= (getSrcValue(addr[1]) << 15);
	// look up the third character, and shift it 20 bits left.
	src |= (getSrcValue(addr[2]) << 10);
	// look up the fourth character, and shift it 20 bits left.
	src |= (getSrcValue(addr[3]) << 5);
	// look up the fifth character, and shift it 20 bits left.
	src |= getSrcValue(addr[4]);
	//printf("asciiToDexcomSrc: val=%u, src=%u\r\n", val, src);
	return src;
}

/* getSrcValue	-	function to determine the encoding value of a character in a Dexcom Transmitter ID.
	Parameters:
		srcVal	-	The character to determine the value of
	Returns:
		uint32	-	The encoding value of the character.
*/
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
	if(command_buff_is("HELLO") || command_buff_is("SAVE"))
	{
		if(command_buff_is("SAVE"))
		{
			//copy flash_settings into the writeBuffer so we can write it to flash
			memcpy(writeBuffer, &flash_settings, sizeof(flash_settings));
			printf("Erasing Flash....\r\n");
			eraseFlash(FLASH_SETTINGS);
			printf("Writing Flash....\r\n");
			writeToFlash(FLASH_SETTINGS, sizeof(flash_settings));
		}
		dexcom_src_to_ascii(flash_settings.dex_tx_id, srcAddr);
		printf_fast("OK WIXEL Dexsim 0.1 (%0x%0x%0x%0x)\r\n", serialNumber[3],serialNumber[2],serialNumber[1],serialNumber[0]);
		printf_fast("TXID: %s\r\n", srcAddr);
		printf_fast_f("ALPHA1: %f\r\n", flash_settings.alpha1);
		printf_fast_f("ALPHA2: %f\r\n", flash_settings.alpha2);
		printf_fast_f("SIGMAA: %f\r\n", flash_settings.sigma_a);
		printf_fast("RAWBG: %lu\r\n", flash_settings.mean_raw_bg);
		return 0;
	}
	if(memcmp(command_buff.commandBuffer, "ALPHA1", 6) == 0)
	{
		if(command_buff.commandBuffer[6] == ' ') 
		{
			flash_settings.alpha1 = atof(&command_buff.commandBuffer[7]);
		}
		printf_fast_f("OK ALPHA1 is %f\r\n Remember to Save\r\n", flash_settings.alpha1);
		return 0;
	}
	if(memcmp(command_buff.commandBuffer, "ALPHA2", 6) == 0)
	{
		if(command_buff.commandBuffer[6] == ' ') 
		{
			flash_settings.alpha2 = atof(&command_buff.commandBuffer[7]);
		}
		printf_fast_f("OK ALPHA2 is %f\r\n Remember to Save\r\n", flash_settings.alpha2);
		return 0;
	}
	if(memcmp(command_buff.commandBuffer, "SIGMAA", 6) == 0)
	{
		if(command_buff.commandBuffer[6] == ' ') 
		{
			flash_settings.sigma_a = atof(&command_buff.commandBuffer[8]);
		}
		printf_fast_f("OK SIGMAA is %f\r\n Remember to Save\r\n", flash_settings.sigma_a);
		return 0;
	}
	if(memcmp(command_buff.commandBuffer, "RAWBG", 5) == 0)
	{
		if(command_buff.commandBuffer[5] == ' ') 
		{
			flash_settings.mean_raw_bg = (atoi(&command_buff.commandBuffer[7])*(RAWBG_FACTOR))+RAWBG_OFFSET;
		}
		printf_fast("OK RAWBG is %lu (%u mg/dl, %0.2f mmol/l)\r\n Remember to Save\r\n", 
		(int)((flash_settings.mean_raw_bg-RAWBG_OFFSET)/RAWBG_FACTOR), 
		((flash_settings.mean_raw_bg-RAWBG_OFFSET)/RAWBG_FACTOR)/18.02);
		return 0;
	}
	if(memcmp(command_buff.commandBuffer, "TXID", 4) == 0)
	{
		if(command_buff.commandBuffer[4] == ' ') 
		{
			//we are being given a TX ID integer (already encoded)
			//printf("%s\r\n",&command_buff.commandBuffer[5]);
			//dex_tx_id = atol(&command_buff.commandBuffer[5]);
			flash_settings.dex_tx_id = asciiToDexcomSrc(&command_buff.commandBuffer[5]);
			//copy flash_settings into the writeBuffer so we can write it to flash
			memcpy(writeBuffer, &flash_settings, sizeof(flash_settings));
			//printf("Erasing....\r\n");
			eraseFlash(FLASH_SETTINGS);
			//printf("Writing....\r\n");
			writeToFlash(FLASH_SETTINGS, sizeof(flash_settings));
			//printf("FLASH_TX_ID has %lu\r\n", *(uint32 XDATA *)FLASH_TX_ID); 
			printf_fast("Flash Saved.\r\n");
		}
		dexcom_src_to_ascii(flash_settings.dex_tx_id, srcAddr);
		printf_fast("OK TXID is %s (%lu)\r\n", srcAddr, flash_settings.dex_tx_id);
		return 0;
	}
	printf("Unrecognised command\r\n");
	return 1;
}

// Process any commands on either UART0 or USB COM.
int controlProtocolService()
{
	static uint32 cmd_to;
	// ok this is where we check if there's anything happening incoming on the USB COM port or UART 0 port.
	int nRet = 1;
	uint8 b = 0;
	//if we have timed out waiting for a command, clear the command buffer and return.
	if(command_buff.nCurReadPos > 0 && (getMs() - cmd_to) > 2000) 
	{
		printf("TIMEOUT\r\n");
		// clear command buffer if there was anything
		init_command_buff(&command_buff);
		return nRet;
	}	
	//while we have something in either buffer,
	while(usbComRxAvailable())
	{
		// if it is the USB, get the bye from it, otherwise get it from the UART.
		if (usbComRxAvailable()) {
			b = usbComRxReceiveByte();
		}
		putchar(b);
		// if it is a CR or LF character, we need to process the command
		if(b == '\r' || b == '\n')
		{
			// ok we got the end of a command;
			putchar(b);
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
		else if(command_buff.nCurReadPos < USB_COMMAND_MAXLEN)
		{
			command_buff.commandBuffer[command_buff.nCurReadPos++]=b;
			// reset the command timeout.
			cmd_to = getMs();
			//printf("buf: %s, %lu\n",command_buff.commandBuffer, cmd_to);
		}
		else
		{
	
			// clear command if there was anything
			init_command_buff(&command_buff);
		}
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
	//radioComTxService();
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


// you can uncomment this if you want a glowing yellow LED when a terminal program is connected
// to the USB.  I got sick of it.
// LineStateChangeCallback - sets the yellow LED to the state of DTR on the USB, whenever it changes.
void LineStateChangeCallback(uint8 state)
{
	usb_connected = state & ACM_CONTROL_LINE_DTR;
}

void waitDoingServices (uint32 wait_time, volatile BIT break_flag, BIT bProtocolServices ) {
	uint32 start_wait;
	start_wait = getMs();
	while ((getMs() - start_wait) < wait_time) {
		doServices(bProtocolServices);
		if(break_flag) return;
		delayMs(20);
	}
}

float random() {
	return 	(float)randomNumber()/255.0;
}
uint16 calculateNextSample() {
	float result = 0.0;
	printf_fast("Calculating next sample\r\n");
	result = powf((float)sample[1], flash_settings.alpha1);
	result = result * powf((float)sample[0], flash_settings.alpha2);
	result = result * powf((float)flash_settings.mean_raw_bg, (1-flash_settings.alpha1-flash_settings.alpha2));
	result = result * expf(flash_settings.sigma_a*((random()+random()+random()+random()+random()+random()-3)/0.7));
	sample[2]=sample[1];
	sample[1]=sample[0];
	sample[0]=(uint16)result;
	printf_fast("n: %u, n-1: %u, n-2: %u", sample[0], sample[1], sample[2]);
	return sample[0];
	
}

void sendPacket(uint8 channel, Dexcom_packet * pkt) {
	XDATA uint8 buffer[sizeof(Dexcom_packet)];
	XDATA uint8 i;
	swap_channel(nChannels[channel], fOffset[channel]);
	memcpy(&buffer, &pkt, sizeof(pkt));
	for(i = 0; i <= sizeof(Dexcom_packet); i++) {
		while(radioComTxAvailable()==0){
			radioComTxService();
			doServices(1);
		}
		radioComTxSendByte(buffer[i]);
	}	
}

void main()
{   
	Dexcom_packet Pkt;
	//initialise the system
	systemInit();
	//initialise Anlogue Input 0
	P0INP = 0x1;
	//initialise the USB port
	usbInit();
	//initialise the dma channel for working with flash.
	dma_Init();
	usbComRequestLineStateChangeNotification(LineStateChangeCallback);
	// we actually only use USB, so no point wasting power on UART
	init_command_buff(&command_buff);
	
	setRadioRegistersInitFunc(dex_RadioSettings);

//	radioQueueInit();
//    radioQueueAllowCrcErrors = 1;
	//initialise the random number generator
	randomSeedFromSerialNumber();
	// these are reset in radioQueueInit and radioMacInit after our init func was already called
	memcpy(&flash_settings, (__xdata *)FLASH_SETTINGS, sizeof(flash_settings));
	if(flash_settings.dex_tx_id == 0xFFFFFFFF) {
		flash_settings.dex_tx_id = 1117317;  //serial number "12345"
		flash_settings.alpha1 = -0.6493;
		flash_settings.alpha2 = 1.6442;
		flash_settings.sigma_a = 0.05;
		flash_settings.mean_raw_bg = 103714+32000;
		writeToFlash(FLASH_SETTINGS,sizeof(flash_settings));
	}
	waitDoingServices(30000,0,0);
	printf_fast("Starting...\r\n");
	//preload the sample array
	sample[0]=flash_settings.mean_raw_bg;
	sample[1]=flash_settings.mean_raw_bg;
	sample[2]=flash_settings.mean_raw_bg;
	printf_fast("Samples loaded with n: %u, n-1: %u, n-2: %u\r\n", sample[0], sample[1], sample[2]);
	memset(&Pkt, 0, sizeof(Dexcom_packet));
	while (1)
	{
		uint8 i;
		Pkt.raw = dex_num_encoder(calculateNextSample());
		Pkt.filtered = dex_num_encoder(sample[2]/2);
		Pkt.battery = 215;
		Pkt.src_addr = flash_settings.dex_tx_id;
		Pkt.dest_addr = 0xFFFF;
		Pkt.unknown = 0;
		Pkt.device_info = 3;
		Pkt.port = 63;
		Pkt.len = sizeof(Pkt);
		if(Pkt.txId >=255)
			Pkt.txId = 0;
	/*
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
} Dexcom_packet;
*/
		printf_fast("about to send a packet\r\n");
		for( i=0; i<4 ; i++) {
			printf("Sending Packet at %lu ms: Length %u, Src %lu, Dst %lu, Raw %u, Filtered %u, Battery %u, Txid %u\r\n", 
				getMs(), 
				Pkt.len, 
				Pkt.src_addr, 
				Pkt.dest_addr, 
				Pkt.raw, 
				Pkt.filtered, 
				Pkt.battery, Pkt.txId
			); 
			sendPacket(i, &Pkt);
			waitDoingServices(500,0,0);
			Pkt.txId++;
		}
		waitDoingServices(298000,0,0);
	}
}


