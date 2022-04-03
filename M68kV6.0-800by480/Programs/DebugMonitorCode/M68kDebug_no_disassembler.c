#include "DebugMonitor.h"
#include "CanBus.h"
// #include "math.h"

//defintions used
#define ByteLength 0x8 //8 bits
#define WordLength 0x10 //16 bits
#define LongWordLength 0x20 //32 bits
#define addressStart 0x08000000
#define addressTestLower 0x09000000
#define addressTestUpper 0x09800000
#define addressTestArbitrary 0x0802f000
#define addressTestRomL 0x00000000
#define addressTestRomH 0x00000007

// use 08030000 for a system running from sram or 0B000000 for system running from dram
// #define StartOfExceptionVectorTable 0x08030000
#define StartOfExceptionVectorTable 0x0B000000

// use 0C000000 for dram or hex 08040000 for sram
// #define TopOfStack 0x08040000
#define TopOfStack 0x0C000000

/*************************************************************
** SPI Controller registers
**************************************************************/
// SPI Registers
#define SPI_Control         (*(volatile unsigned char *)(0x00408020))
#define SPI_Status          (*(volatile unsigned char *)(0x00408022))
#define SPI_Data            (*(volatile unsigned char *)(0x00408024))
#define SPI_Ext             (*(volatile unsigned char *)(0x00408026))
#define SPI_CS              (*(volatile unsigned char *)(0x00408028))
// these two macros enable or disable the flash memory chip enable off SSN_O[7..0]
// in this case we assume there is only 1 device connected to SSN_O[0] so we can
// write hex FE to the SPI_CS to enable it (the enable on the flash chip is active low)
// and write FF to disable it
#define   Enable_SPI_CS()             SPI_CS = 0xFE
#define   Disable_SPI_CS()            SPI_CS = 0xFF

//commands for spi
#define PageProgram 0x02
#define Read 0x03
#define ReadStatus 0x05
#define WriteEnable 0x06
#define EraseChip 0x60

#define KbToBytes256 262144

#define DUMMY 0x33 // used for the read commands

/*************************************************************
** I2C CORE Controller registers // switched these to correspond to the doc description
**************************************************************/
#define I2C_CORE_PRESCALE_LO     (*(volatile unsigned char *)(0x00408000))
#define I2C_CORE_PRESCALE_HI     (*(volatile unsigned char *)(0x00408002))  
#define I2C_CORE_CONTROL         (*(volatile unsigned char *)(0x00408004))
#define I2C_CORE_TRANSMIT        (*(volatile unsigned char *)(0x00408006))
#define I2C_CORE_RECEIVE         (*(volatile unsigned char *)(0x00408006))
#define I2C_CORE_COMMAND         (*(volatile unsigned char *)(0x00408008))
#define I2C_CORE_STATUS          (*(volatile unsigned char *)(0x00408008))

// Commands to be put in the command register of the I2C CORE Contoller I2C_CORE_COMMAND
// |STA|STO|RD |WR |ACK |R  |R  |IACK|
// |   |   |   |   |    |   |   |    |
// by default, clear interrupt acknowledges all the time (instructed by doc)
#define I2C_SlaveReadStart  0xA9       // (0b10101001) // read bit set, start bit set
#define I2C_SlaveReadStartStop  0xE1   // (0b11101001) // read bit set, start bit set
#define I2C_SlaveRead       0x29       // (0b00101001) // read bit set
#define I2C_SlaveReadStop   0x69       //(0b01101001)  // read bit set, stop bit set

#define I2C_SlaveWriteStart 0x99       // (0b10011001) // write bit set, start bit set
#define I2C_SlaveWrite      0x19       // (0b00011001) // write bit set
#define I2C_SlaveWriteStop  0x59       // (0b01011001) // write bit set, stop bit set

#define I2C_SlaveStop       0x49       // (0b01001001) //
#define I2C_SlaveStart      0x89       // (0b10001001) // 

#define I2C_setIACK         0x09       // (0b00001001) // nothing but clear IACK

#define I2C_MasterAck       0x01       // (0b00000001)
#define I2C_MasterNOAck     0x00       // (0b00000000)
#define I2C_SlaveReadAck    0x21       // (0b00100001) // read bit set

#define I2CReadStop 0x41

// definitions for the I2C EEPROM module
#define I2C_EEPROM_CTRLBYTE_LO_BLOCK_READ   0xA1  // 0b10100001 // lsb 1 means read from slave
#define I2C_EEPROM_CTRLBYTE_HI_BLOCK_READ   0xA9  // 0b10101001 // lsb 1 means read from slave
#define I2C_EEPROM_CTRLBYTE_LO_BLOCK_WRITE  0xA0  // 0b10100000 // lsb 0 means write to slave
#define I2C_EEPROM_CTRLBYTE_HI_BLOCK_WRITE  0xA8  // 0b10101000 // lsb 0 means write to slave

// limit definition for block write:
#define I2C_EEPROM_BLOCK_SIZE_LIMIT 0x1F400
#define I2C_EEPROM_BLOCK_FLIPPER 0x08

#define ADC_SLAVE_ADDRESS_MWRITE 0x92
#define ADC_SLAVE_ADDRESS_MREAD 0x93
#define ADC_ANALOG_OUT_CTRL_BYTE 0x40
#define ADC_ANALOG_READ_CTRL_BYTE 0x44 // changed that

#define PI_VAL 3



/********************************************************************


/* DO NOT INITIALISE GLOBAL VARIABLES - DO IT in MAIN() */

unsigned int i, x, y, z, PortA_Count;
int     Trace, GoFlag, Echo;                       // used in tracing/single stepping

// 68000 register dump and preintialise value (these can be changed by the user program when it is running, e.g. stack pointer, registers etc

unsigned int d0,d1,d2,d3,d4,d5,d6,d7 ;
unsigned int a0,a1,a2,a3,a4,a5,a6 ;
unsigned int PC, SSP, USP ;
unsigned short int SR;

// Breakpoint variables
unsigned int BreakPointAddress[8];                      //array of 8 breakpoint addresses
unsigned short int BreakPointInstruction[8] ;           // to hold the instruction opcode at the breakpoint
unsigned int BreakPointSetOrCleared[8] ;
unsigned int InstructionSize ;

// watchpoint variables
unsigned int WatchPointAddress[8];                      //array of 8 breakpoint addresses
unsigned int WatchPointSetOrCleared[8] ;
char WatchPointString[8][100] ;

char    TempString[100] ;


//added by Warren
char datatypeselection(void);            // allows the user to choose the length of data (byte, word, long word)
char testdataselection(void);            // allows the user to choose the data pattern
void populatememory(int, char);      // populate the memory with selected data length and pattern
void checkmemory(int, char);         // check the memory with selected data length and pattern
void automatictests(void);
void manualtests(void);
void memorywrite(int, int, int, char);
void memoryread(int, int, char);
int useraddressStart(void);
int useraddressEnd(void);
/************************************************************************************
*Subroutine to give the 68000 something useless to do to waste 1 mSec
************************************************************************************/
void Wait1ms(void)
{
    long int  i ;
    for(i = 0; i < 1000; i ++)
        ;
}

/************************************************************************************
*Subroutine to give the 68000 something useless to do to waste 3 mSec
**************************************************************************************/
void Wait3ms(void)
{
    int i ;
    for(i = 0; i < 3; i++)
        Wait1ms() ;
}

/*********************************************************************************************
*Subroutine to initialise the display by writing some commands to the LCD internal registers
*********************************************************************************************/
void Init_LCD(void)
{
    LCDcommand = (char)(0x0c) ;
    Wait3ms() ;
    LCDcommand = (char)(0x38) ;
    Wait3ms() ;
}

/******************************************************************************
*subroutine to output a single character held in d1 to the LCD display
*it is assumed the character is an ASCII code and it will be displayed at the
*current cursor position
*******************************************************************************/
void Outchar(int c)
{
    LCDdata = (char)(c);
    Wait1ms() ;
}

/**********************************************************************************
*subroutine to output a message at the current cursor position of the LCD display
************************************************************************************/
void OutMess(char *theMessage)
{
    char c ;
    while((c = *theMessage++) != (char)(0))
        Outchar(c) ;
}

/******************************************************************************
*subroutine to clear the line by issuing 24 space characters
*******************************************************************************/
void Clearln(void)
{
    unsigned char i ;
    for(i = 0; i < 24; i ++)
        Outchar(' ') ;  /* write a space char to the LCD display */
}

/******************************************************************************
*subroutine to move the cursor to the start of line 1 and clear that line
*******************************************************************************/
void Oline0(char *theMessage)
{
    LCDcommand = (char)(0x80) ;
    Wait3ms();
    Clearln() ;
    LCDcommand = (char)(0x80) ;
    Wait3ms() ;
    OutMess(theMessage) ;
}

/******************************************************************************
*subroutine to move the cursor to the start of line 2 and clear that line
*******************************************************************************/
void Oline1(char *theMessage)
{
    LCDcommand = (char)(0xC0) ;
    Wait3ms();
    Clearln() ;
    LCDcommand = (char)(0xC0) ;
    Wait3ms() ;
    OutMess(theMessage) ;
}

void InstallExceptionHandler( void (*function_ptr)(), int level)
{
    volatile long int *RamVectorAddress = (volatile long int *)(StartOfExceptionVectorTable) ;   // pointer to the Ram based interrupt vector table created in Cstart in debug monitor

    RamVectorAddress[level] = (long int *)(function_ptr);
}


void TestLEDS(void)
{
    int delay ;
    unsigned char count = 0 ;

    while(1)    {
        PortA = PortB = PortC = PortD = HEX_A = HEX_B = HEX_C = HEX_D = ((count << 4) + (count & 0x0f)) ;
        for(delay = 0; delay < 200000; delay ++)
            ;
        count ++;
    }
}

void SwitchTest(void)
{
    int i, switches = 0 ;

	printf("\r\n") ;

    while(1)    {
        switches = (PortB << 8) | (PortA) ;
        printf("\rSwitches SW[7-0] = ") ;
        for( i = (int)(0x00000080); i > 0; i = i >> 1)  {
            if((switches & i) == 0)
                printf("0") ;
            else
                printf("1") ;
        }
    }
}

/*********************************************************************************************
*Subroutine to initialise the RS232 Port by writing some commands to the internal registers
*********************************************************************************************/
void Init_RS232(void)
{
    RS232_Control = (char)(0x15) ; //  %00010101    divide by 16 clock, set rts low, 8 bits no parity, 1 stop bit transmitter interrupt disabled
    RS232_Baud = (char)(0x1) ;      // program baud rate generator 000 = 230k, 001 = 115k, 010 = 57.6k, 011 = 38.4k, 100 = 19.2, all others = 9600
}

int kbhit(void)
{
    if(((char)(RS232_Status) & (char)(0x01)) == (char)(0x01))    // wait for Rx bit in status register to be '1'
        return 1 ;
    else
        return 0 ;
}

/*********************************************************************************************************
**  Subroutine to provide a low level output function to 6850 ACIA
**  This routine provides the basic functionality to output a single character to the serial Port
**  to allow the board to communicate with HyperTerminal Program
**
**  NOTE you do not call this function directly, instead you call the normal putchar() function
**  which in turn calls _putch() below). Other functions like puts(), printf() call putchar() so will
**  call _putch() also
*********************************************************************************************************/

int _putch( int c)
{
    while(((char)(RS232_Status) & (char)(0x02)) != (char)(0x02))    // wait for Tx bit in status register or 6850 serial comms chip to be '1'
        ;

    (char)(RS232_TxData) = ((char)(c) & (char)(0x7f));                      // write to the data register to output the character (mask off bit 8 to keep it 7 bit ASCII)
    return c ;                                              // putchar() expects the character to be returned
}

/*********************************************************************************************************
**  Subroutine to provide a low level input function to 6850 ACIA
**  This routine provides the basic functionality to input a single character from the serial Port
**  to allow the board to communicate with HyperTerminal Program Keyboard (your PC)
**
**  NOTE you do not call this function directly, instead you call the normal _getch() function
**  which in turn calls _getch() below). Other functions like gets(), scanf() call _getch() so will
**  call _getch() also
*********************************************************************************************************/

int _getch( void )
{
    int c ;
    while(((char)(RS232_Status) & (char)(0x01)) != (char)(0x01))    // wait for Rx bit in 6850 serial comms chip status register to be '1'
        ;

    c = (RS232_RxData & (char)(0x7f));                   // read received character, mask off top bit and return as 7 bit ASCII character

    // shall we echo the character? Echo is set to TRUE at reset, but for speed we don't want to echo when downloading code with the 'L' debugger command
    if(Echo)
        _putch(c);

    return c ;
}

// flush the input stream for any unread characters

void FlushKeyboard(void)
{
    char c ;

    while(1)    {
        if(((char)(RS232_Status) & (char)(0x01)) == (char)(0x01))    // if Rx bit in status register is '1'
            c = ((char)(RS232_RxData) & (char)(0x7f)) ;
        else
            return ;
     }
}

// converts hex char to 4 bit binary equiv in range 0000-1111 (0-F)
// char assumed to be a valid hex char 0-9, a-f, A-F

char xtod(int c)
{
    if ((char)(c) <= (char)('9'))
        return c - (char)(0x30);    // 0 - 9 = 0x30 - 0x39 so convert to number by sutracting 0x30
    else if((char)(c) > (char)('F'))    // assume lower case
        return c - (char)(0x57);    // a-f = 0x61-66 so needs to be converted to 0x0A - 0x0F so subtract 0x57
    else
        return c - (char)(0x37);    // A-F = 0x41-46 so needs to be converted to 0x0A - 0x0F so subtract 0x37
}

int Get2HexDigits(char *CheckSumPtr)
{
    register int i = (xtod(_getch()) << 4) | (xtod(_getch()));

    if(CheckSumPtr)
        *CheckSumPtr += i ;

    return i ;
}

int Get4HexDigits(char *CheckSumPtr)
{
    return (Get2HexDigits(CheckSumPtr) << 8) | (Get2HexDigits(CheckSumPtr));
}

int Get6HexDigits(char *CheckSumPtr)
{
    return (Get4HexDigits(CheckSumPtr) << 8) | (Get2HexDigits(CheckSumPtr));
}

int Get8HexDigits(char *CheckSumPtr)
{
    return (Get4HexDigits(CheckSumPtr) << 16) | (Get4HexDigits(CheckSumPtr));
}

void DumpMemory(void)   // simple dump memory fn
{
    int i, j ;
    unsigned char *RamPtr,c ; // pointer to where the program is download (assumed)

    printf("\r\nDump Memory Block: <ESC> to Abort, <SPACE> to Continue") ;
    printf("\r\nEnter Start Address: ") ;
    RamPtr = Get8HexDigits(0) ;

    while(1)    {
        for(i = 0; i < 16; i ++)    {
            printf("\r\n%08x ", RamPtr) ;
            for(j=0; j < 16; j ++)  {
                printf("%02X",RamPtr[j]) ;
                putchar(' ') ;
            }

            // now display the data as ASCII at the end

            printf("  ") ;
            for(j = 0; j < 16; j++) {
                c = ((char)(RamPtr[j]) & 0x7f) ;
                if((c > (char)(0x7f)) || (c < ' '))
                    putchar('.') ;
                else
                    putchar(RamPtr[j]) ;
            }
            RamPtr = RamPtr + 16 ;
        }
        printf("\r\n") ;

        c = _getch() ;
        if(c == 0x1b)          // break on ESC
            break ;
     }
}

void FillMemory()
{
    char *StartRamPtr, *EndRamPtr ;
    unsigned char FillData ;

    printf("\r\nFill Memory Block") ;
    printf("\r\nEnter Start Address: ") ;
    StartRamPtr = Get8HexDigits(0) ;

    printf("\r\nEnter End Address: ") ;
    EndRamPtr = Get8HexDigits(0) ;

    printf("\r\nEnter Fill Data: ") ;
    FillData = Get2HexDigits(0) ;
    printf("\r\nFilling Addresses [$%08X - $%08X] with $%02X", StartRamPtr, EndRamPtr, FillData) ;

    while(StartRamPtr < EndRamPtr)
        *StartRamPtr++ = FillData ;
}

void Load_SRecordFile()
{
    int i, Address, AddressSize, DataByte, NumDataBytesToRead, LoadFailed, FailedAddress, AddressFail, SRecordCount = 0, ByteTotal = 0 ;
    int result, ByteCount ;

    char c, CheckSum, ReadCheckSum, HeaderType ;
    char *RamPtr ;                          // pointer to Memory where downloaded program will be stored

    LoadFailed = 0 ;                        //assume LOAD operation will pass
    AddressFail = 0 ;
    Echo = 0 ;                              // don't echo S records during download

    printf("\r\nUse HyperTerminal to Send Text File (.hex)\r\n") ;

    while(1)    {
        CheckSum = 0 ;
        do {
            c = toupper(_getch()) ;

            if(c == 0x1b )      // if break
                return;
         }while(c != (char)('S'));   // wait for S start of header

        HeaderType = _getch() ;

        if(HeaderType == (char)('0') || HeaderType == (char)('5'))       // ignore s0, s5 records
            continue ;

        if(HeaderType >= (char)('7'))
            break ;                 // end load on s7,s8,s9 records

// get the bytecount

        ByteCount = Get2HexDigits(&CheckSum) ;

// get the address, 4 digits for s1, 6 digits for s2, and 8 digits for s3 record

        if(HeaderType == (char)('1')) {
            AddressSize = 2 ;       // 2 byte address
            Address = Get4HexDigits(&CheckSum);
        }
        else if (HeaderType == (char)('2')) {
            AddressSize = 3 ;       // 3 byte address
            Address = Get6HexDigits(&CheckSum) ;
        }
        else    {
            AddressSize = 4 ;       // 4 byte address
            Address = Get8HexDigits(&CheckSum) ;
        }

        RamPtr = (char *)(Address) ;                            // point to download area

        NumDataBytesToRead = ByteCount - AddressSize - 1 ;


        for(i = 0; i < NumDataBytesToRead; i ++) {     // read in remaining data bytes (ignore address and checksum at the end
            DataByte = Get2HexDigits(&CheckSum) ;
            *RamPtr++ = DataByte ;                      // store downloaded byte in Ram at specified address
            ByteTotal++;
        }

// checksum is the 1's complement of the sum of all data pairs following the bytecount, i.e. it includes the address and the data itself

        ReadCheckSum = Get2HexDigits(0) ;

        if((~CheckSum&0Xff) != (ReadCheckSum&0Xff))   {
            LoadFailed = 1 ;
            FailedAddress = Address ;
            break;
        }

        SRecordCount++ ;

        // display feedback on progress
        if(SRecordCount % 25 == 0)
            putchar('.') ;
     }

     if(LoadFailed == 1) {
        printf("\r\nLoad Failed at Address = [$%08X]\r\n", FailedAddress) ;
     }

     else
        printf("\r\nSuccess: Downloaded %d bytes\r\n", ByteTotal) ;

     // pause at the end to wait for download to finish transmitting at the end of S8 etc

     for(i = 0; i < 400000; i ++)
        ;

     FlushKeyboard() ;
     Echo = 1;
}


void MemoryChange(void)
{
    unsigned char *RamPtr,c ; // pointer to memory
    int Data ;

    printf("\r\nExamine and Change Memory") ;
    printf("\r\n<ESC> to Stop, <SPACE> to Advance, '-' to Go Back, <DATA> to change") ;

    printf("\r\nEnter Address: ") ;
    RamPtr = Get8HexDigits(0) ;

    while(1)    {
        printf("\r\n[%08x] : %02x  ", RamPtr, *RamPtr) ;
        c = tolower(_getch()) ;

       if(c == (char)(0x1b))
            return ;                                // abort on escape

       else if((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f')) {  // are we trying to change data at this location by entering a hex char
            Data = (xtod(c) << 4) | (xtod(_getch()));
            *RamPtr = (char)(Data) ;
            if(*RamPtr != Data) {
                printf("\r\nWarning Change Failed: Wrote [%02x], Read [%02x]", Data, *RamPtr) ;
            }
        }
        else if(c == (char)('-'))
            RamPtr -= 2 ; ;

        RamPtr ++ ;
    }
}

/*******************************************************************
** Write a program to SPI Flash Chip from memory and verify by reading back
********************************************************************/

void EraseFlash(void)
{
    Enable_SPI_CS();
    WriteSPIChar(WriteEnable);
    Disable_SPI_CS();

    Enable_SPI_CS();
    WriteSPIChar(EraseChip);
    Disable_SPI_CS();

    Enable_SPI_CS();
    WriteSPIChar(ReadStatus);
    while((WriteSPIChar(DUMMY)&0x01) == 1); // wait until idle
    Disable_SPI_CS();
}
void WriteToFlash(int address24bit, unsigned char data)
{
    unsigned char holder;
    Enable_SPI_CS();
    WriteSPIChar(WriteEnable);
    Disable_SPI_CS();

    Enable_SPI_CS();
    WriteSPIChar(PageProgram);

    
    holder = address24bit>>16;
    WriteSPIChar(holder);
    holder = address24bit>>8;
    WriteSPIChar(holder);
    holder = (unsigned char) address24bit;
    WriteSPIChar(holder);

    WriteSPIChar((int)data);
    Disable_SPI_CS();

    Enable_SPI_CS();
    WriteSPIChar(ReadStatus);
    while((WriteSPIChar(DUMMY)&0x01) == 1) // wait until idle
    {}
    Disable_SPI_CS();
}

unsigned char ReadFromFlash(int address24bit, unsigned char data)
{
    unsigned char holder;
    Enable_SPI_CS();
    WriteSPIChar(Read);

    holder = address24bit>>16;
    WriteSPIChar(holder);
    holder = address24bit>>8;
    WriteSPIChar(holder);
    holder = (unsigned char) address24bit;
    WriteSPIChar(holder);

    data = (unsigned char)WriteSPIChar(DUMMY);
    Disable_SPI_CS();

    return data;
}
void ProgramFlashChip(void)
{
    //
    // TODO : put your code here to program the 1st 256k of ram (where user program is held at hex 08000000) to SPI flash chip
    // TODO : then verify by reading it back and comparing to memory
    //
    // should
    unsigned char * dataPointer = addressStart;
    unsigned char byteHolder;
    unsigned char checkByte;
    int flashAddr = 0;
    int i = 0;
    //erase
    printf("\n\r Erasing flash \n\r");

    EraseFlash();

    printf("Erased Flash \n \r");

    printf("Writing Program to memory \n\r");
    for(i = 0; i < KbToBytes256; i++ )//256kb or 128k 0xnn chars
    {
        byteHolder = *dataPointer;
        dataPointer++;
        checkByte = (unsigned char)WriteToFlash(i,byteHolder);
    }
    printf("Finished writing\n\r");
    dataPointer = addressStart; //validate data

    printf("Starting verification\n\r");
    for (i = 0; i < KbToBytes256; i++)//256kb or 128k 0xnn chars
    {
        byteHolder = *dataPointer;
        dataPointer++;
        checkByte = ReadFromFlash(i, byteHolder);
        if (checkByte != byteHolder)
        {
            printf("Invalid data\n\r");
            printf("Flash Addr = %x Ram Addr = %x\n\r", i, dataPointer);
            printf("Written Byte = %x Expected Byte = %x\n\r", checkByte, byteHolder);
            break;
        }
        flashAddr = i;

    }

    printf("Last Ram Addr written = %x Last Flash Addr wrriten = %x \n\r",flashAddr,dataPointer);
    printf("Byte at Ram = %x Byte at Flash = %x\n\r", byteHolder, checkByte);
    printf("Finished verification all data is correct \n\r");

}

/*************************************************************************
** Load a program from SPI Flash Chip and copy to Dram
**************************************************************************/
void LoadFromFlashChip(void)
{
    //
    // TODO : put your code here to read 256k of data from SPI flash chip and store in user ram starting at hex 08000000
    //

    // variables used to interface with the SPI
    unsigned int SPI_address;
	unsigned int endSPI_address;
	unsigned char firstSPIAddress;
	unsigned int index;
    char received_byte; // used to store the read data from the SPI

    // byte pointer used to write to ram
    unsigned char* ram_address_pointer;

    printf("\r\nLoading Program From SPI Flash....\n\r") ; // this was already included

	SPI_address = 0;                        // used as an index in the for loop
	endSPI_address = 256000;                // used to determine the last address to be read in SPI
	ram_address_pointer = addressStart;     // set the byte sized pointer
	firstSPIAddress = 0x00;

	// initiate the read command
	// followed by the 24 bit address, which is zero in our case
    Enable_SPI_CS();
	WriteSPIChar(Read);
	for (index = 0; index < 3; index++){
		WriteSPIChar(firstSPIAddress);
    }

    // at this point, we should be able to read from the SPI without interruption
    // can use the data read from the SPI to write to ram in a loop
    for (SPI_address = 0; SPI_address<=endSPI_address; SPI_address++){
        received_byte = WriteSPIChar(DUMMY);    // write a dummy byte to the SPI to receive read data
        *ram_address_pointer = received_byte;   // writing to RAM
        ram_address_pointer++;                  // incrementing the ram address accessed for next write
    }

    Disable_SPI_CS(); // ends the read process

    printf("Succesfully loaded\n\r");

    return;
}

/******************************************************************************************
** The following code is for the SPI controller
*******************************************************************************************/
// return true if the SPI has finished transmitting a byte (to say the Flash chip) return false otherwise
// this can be used in a polling algorithm to know when the controller is busy or idle.
int TestForSPITransmitDataComplete(void)    {
    /* TODO replace 0 below with a test for status register SPIF bit and if set, return true */
    int SPIFBit = (SPI_Control>>7);
    return SPIFBit;
}
/************************************************************************************
** initialises the SPI controller chip to set speed, interrupt capability etc.
************************************************************************************/
void SPI_Init(void)
{
    //TODO
    //
    // Program the SPI Control, EXT, CS and Status registers to initialise the SPI controller
    // Don't forget to call this routine from main() before you do anything else with SPI
    //
    // Here are some settings we want to create
    //
    // Control Reg     - interrupts disabled, core enabled, Master mode, Polarity and Phase of clock = [0,0], speed =  divide by 32 = approx 700Khz
    // Ext Reg         - in conjunction with control reg, sets speed above and also sets interrupt flag after every completed transfer (each byte)
    // SPI_CS Reg      - control selection of slave SPI chips via their CS# signals
    // Status Reg      - status of SPI controller chip and used to clear any write collision and interrupt on transmit complete flag
    SPI_Control = (unsigned char) 0x53;//01_10000
    SPI_Ext = (unsigned char) 0x00;//00____00
    SPI_Status = (unsigned char) 0xC0;//1100_0000
    Disable_SPI_CS();
    printf("\r\nFinished Init of SPI\r\n");

}
/************************************************************************************
** return ONLY when the SPI controller has finished transmitting a byte
************************************************************************************/
void WaitForSPITransmitComplete(void)
{
    // TODO : poll the status register SPIF bit looking for completion of transmission
    // once transmission is complete, clear the write collision and interrupt on transmit complete flags in the status register (read documentation)
    // just in case they were set
    while(!(SPI_Status & 0x80));

    SPI_Status = 0xC0;//1100_0000
}
/************************************************************************************
** Write a byte to the SPI flash chip via the controller and returns (reads) whatever was
** given back by SPI device at the same time (removes the read byte from the FIFO)
************************************************************************************/
char WriteSPIChar(char c)
{
    // todo - write the byte in parameter 'c' to the SPI data register, this will start it transmitting to the flash device
    // wait for completion of transmission
    // return the received data from Flash chip (which may not be relevent depending upon what we are doing)
    // by reading fom the SPI controller Data Register.
    // note however that in order to get data from an SPI slave device (e.g. flash) chip we have to write a dummy byte to it
    //
    // modify '0' below to return back read byte from data register
    //

    SPI_Data = c;
    WaitForSPITransmitComplete();

    return SPI_Data;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
// ADDED CODE FOR LAB 5 - Interfacing with peripherals via I2C via I2C master core from OpenCores

// TO DO List:




void I2C_Init(void) {
    // don't forget to initialize and enable the IIC controller
    // Set it up for no interrupts 
    // set the clock frequency for 100KHz (see equation in the IIC controller datasheet page 7)
    // Connect the controller to the 25 MHz clock (since we are using the non-cached version of the lab) (this is done already in schematics I believe)

    // question: should we have the enable always on as set by the init? or should we clear the EN bit when no transfers are being performed (check page 8 of i2c core documentation)
    // PLease note that all reserved bits are read as zeros. To ensure forward compatibility, they should be written as zeros.
    //                                          76543210
    I2C_CORE_CONTROL = (unsigned char) 0x80; // 10000000
    
    // need to change the prescaling
    // prescale = (clock frequency/(5*desired frequency))-1 = 49 (decimal) = 31 (hex) = 00110001 (binary)
    I2C_CORE_PRESCALE_HI = (unsigned char) 0x00;
    I2C_CORE_PRESCALE_LO = (unsigned char) 0x31;

    I2C_CORE_COMMAND = I2C_SlaveStart;
    printf("\r\nFinished Init of I2C\r\n");
}

void I2C_wait_transmit_finish(void)
{   
    int delay;
    // polling the bit 1 of the status register for transmit complete
    // '1' when transferring data
    // '0' when transfer complete
    
    // printf("\r\nWaiting for negation of Transaction in Progress Byte\n\r");
    for (delay = 0; delay<250; delay++);

    // while(I2C_CORE_STATUS & 0x02 == 0x02);  // unlike checkign the ack, I think this one is fine if it hangs the bus?
    // printf("\r\nTIP negated\n\r");
    // return;
    // TIP
    //xxxxxxax
    //00000010
    //000000a0

    //xxxxxxxb
    //00000001
    //00000001 
    while (1) {
        if(!(I2C_CORE_STATUS & 0x02 == 0x02)){
            //printf("\r\nNo transaction in progress\n\r");
            break;
        }
        if((I2C_CORE_STATUS & 0x01 == 0x01)){
            //printf("\r\nByte transfer has been completed\n\r");
            break;
        }
    }
}

void I2C_check_slave_acknowledge(void)
{
    int delay;
    // polling the bit 7 of the status register for acknowledge
    // '1' = no acknowedlge received
    // '0' = acknowledge received

    for (delay = 0; delay<250; delay++);
    // printf("\r\nWaiting for acknowledge\n\r");
    while ((I2C_CORE_STATUS & 0x80) == 0x80);// while no acknowledge
    // if ((I2C_CORE_STATUS & 0x80) == 1){
    //     // no acknowledge
    //     printf("\r\nERROR: no acknowledge from slave\n\r"); // this might not be sufficient, might need to poll for the acknowledge, by sending the start bit and control byte over and over
    // }
    // 76543210
    // axxxxxxx
    // 10000000
    //if a = 1
    // 10000000

}

//****************************************************************
// 1. a function to Read a SINGLE BYTE from the EEPROM from a specified starting address anywhere in the chip (this is a random read)
void I2C_EEPROM_RandomRead(void) // no parameters passed, no value returned
{
    //printf("\r\nRandom Read operation (function 1)\n\r");
    
    unsigned char control_byte;
    unsigned char read_byte;
    unsigned int user_internal_address_complete;
    unsigned char internal_address_hi = 0;
    unsigned char internal_address_lo = 0;

    //read_byte = I2C_CORE_RECEIVE;

    //printf("\r\nPrior to read, data in receive is %x\n",read_byte);

    printf("\r\nENTER READ ADDR (00000 - 01F3FF):\n\r"); // to print the received byte.
    user_internal_address_complete = Get6HexDigits(0);

    if(user_internal_address_complete > 0x1f3ff){
        printf("\r\nInvalid address, default address used: 000000\n\r");
        user_internal_address_complete = 0;
    }

    // parse the address into bytes that can be sent to the EEPROM
    // check that the address is within the first block or the second block
    // the 512Kbit internal limitation
    // 512Kbit == 64000 bytes -> from 0 to 63999(0xF9FF)
    //11111001111111111
    if(user_internal_address_complete > 0xF9FF){
        // we want to access the higher block (set block bit to 1)
        user_internal_address_complete = user_internal_address_complete - 0xFA00;
        internal_address_hi = user_internal_address_complete >> 8;
        internal_address_lo = user_internal_address_complete;
        control_byte = I2C_EEPROM_CTRLBYTE_HI_BLOCK_WRITE; // write ebecause we need to write the address first
    }
    else{
        // we want to acess the lower block (set block bit to 0) 
        internal_address_hi = user_internal_address_complete >> 8;
        internal_address_lo = user_internal_address_complete;
        control_byte = I2C_EEPROM_CTRLBYTE_LO_BLOCK_WRITE; // write because we need to write the address first
    }

    //printf("\r\nHi address byte: %x\nlo address byte: %x\n", internal_address_hi, internal_address_lo); // remove later, just for debug purposes

    // check that device is ready
    I2C_wait_transmit_finish();
    // write the slave address
    I2C_CORE_TRANSMIT = control_byte; // this contains the slave address,
    // I2C_CORE_COMMAND =  I2C_SlaveStart;
    // I2C_CORE_COMMAND =  I2C_SlaveWrite;
    I2C_CORE_COMMAND = I2C_SlaveWriteStart;
    I2C_wait_transmit_finish();
    I2C_check_slave_acknowledge();

    I2C_CORE_TRANSMIT = internal_address_hi;
    I2C_CORE_COMMAND = I2C_SlaveWrite;
    I2C_wait_transmit_finish();
    I2C_check_slave_acknowledge();

    I2C_CORE_TRANSMIT = internal_address_lo;
    I2C_CORE_COMMAND = I2C_SlaveWrite;
    I2C_wait_transmit_finish();
    I2C_check_slave_acknowledge();

    I2C_CORE_TRANSMIT = control_byte | 0x01;
    I2C_CORE_COMMAND = I2C_SlaveWriteStart;
    I2C_wait_transmit_finish();
    I2C_check_slave_acknowledge();
    // I2C_CORE_COMMAND =  I2C_SlaveStart;
    // I2C_CORE_COMMAND =  I2C_SlaveRead;
    I2C_CORE_COMMAND = I2C_SlaveRead; // maybe have to include stop in this line directly 
    I2C_wait_transmit_finish();
    //I2C_CORE_COMMAND = I2C_SlaveReadStartStop;

    // poll status to know when we can read from receive register
    while(I2C_CORE_STATUS & 0x01 == 0x00);// will stay here unless the core status bit 0 is set to 1
    
    // I2C_CORE_COMMAND = I2C_setIACK; // reset the flag above

    read_byte = I2C_CORE_RECEIVE;
    I2C_CORE_COMMAND = I2C_SlaveStop;// this also rests the flag above

    printf("Byte read at address hex %x%x is %x\n",internal_address_hi, internal_address_lo ,read_byte);
    printf("Random Read operation complete\n");
    
}

//****************************************************************
// 2. A function to Write a SINGLE BYTE to the EEPROM from a specified starting address anywhere in the chip
void I2C_EEPROM_RandomWrite(void) // no parameters passed, no value returned
{
    // printf("\r\nRandom Write operation (function 2)\n\r"); 
    unsigned char control_byte;
    unsigned char byte_to_write;
    unsigned int user_internal_address_complete;
    unsigned char internal_address_hi;
    unsigned char internal_address_lo;

    printf("\r\nENTER WRITE ADDR (00000 - 01F3FF):\n\r"); // to print the received byte.
    user_internal_address_complete = Get6HexDigits(0);

    if(user_internal_address_complete > 0x1f3ff){
        printf("\r\nInvalid address, default address used: 000000\n\r");
        user_internal_address_complete = 0;
    }

    if(user_internal_address_complete > 0xF9FF){
        // we want to access the higher block (set block bit to 1)
        user_internal_address_complete = user_internal_address_complete - 0xFA00;
        internal_address_hi = user_internal_address_complete >> 8;
        internal_address_lo = user_internal_address_complete;
        
        control_byte = I2C_EEPROM_CTRLBYTE_HI_BLOCK_WRITE; // write ebecause we need to write the address first
    }
    else{
        // we want to acess the lower block (set block bit to 0) 
        internal_address_hi = user_internal_address_complete >> 8;
        internal_address_lo = user_internal_address_complete;
        control_byte = I2C_EEPROM_CTRLBYTE_LO_BLOCK_WRITE; // write because we need to write the address first
    }

    //printf("\r\nHi address byte: %x\nlo address byte: %x\n", internal_address_hi, internal_address_lo); // remove later, just for debug purposes
        
    printf("\r\nEnter HEX byte data to write\n\r");
    byte_to_write = Get2HexDigits(0);

    //printf("\r\nByte to be written: %x\n", byte_to_write); // remove later, just for debug purposes

    // check that device is ready
    I2C_wait_transmit_finish();
    // write the slave address
    //printf("\r\nFirst byte sent: %x\n", control_byte); // remove later, just for debug purposes
    I2C_CORE_TRANSMIT = control_byte; // this contains the slave address,
    // I2C_CORE_COMMAND =  I2C_SlaveStart;
    // I2C_CORE_COMMAND =  I2C_SlaveWrite;
    I2C_CORE_COMMAND = I2C_SlaveWriteStart;
    I2C_wait_transmit_finish();
    I2C_check_slave_acknowledge();

    I2C_CORE_TRANSMIT = internal_address_hi;
    I2C_CORE_COMMAND = I2C_SlaveWrite;
    I2C_wait_transmit_finish();
    I2C_check_slave_acknowledge();

    I2C_CORE_TRANSMIT = internal_address_lo;
    I2C_CORE_COMMAND = I2C_SlaveWrite;
    I2C_wait_transmit_finish();
    I2C_check_slave_acknowledge();

    I2C_CORE_TRANSMIT = byte_to_write;
    I2C_CORE_COMMAND = I2C_SlaveWriteStop;
    I2C_wait_transmit_finish();
    I2C_check_slave_acknowledge();

    printf("\r\nRandom Write operation complete, check result with random read\n"); 
}


//****************************************************************
//3. A function to Read a BLOCK of any specified size up to 128k bytes starting at ANY address in the EEPROM chip. (this will take some thinking)

void I2C_EEPROM_BLOCKREAD(void){
    unsigned char control_byte;
    unsigned char read_byte;
    unsigned int user_internal_address_complete;
    unsigned int user_block_size;                   // this would be up to 1F400
    unsigned char internal_address_hi;
    unsigned char internal_address_lo;
    unsigned int next_address_tracker;                   // keeps track of the internal address so that we can know when to switch the blocks
    unsigned int total_bytes_tracker;               // keeps track of how close we are to the user_block_size
    unsigned int bytes_counter;                     // keeps track of when the 128 bytes have been written over so the current read can be ended and a new one started
    unsigned int change_block_flag;                 // flag used to know when to send out the new control and address

    unsigned int block_flag; // flag used to know what block we are in to print the correct address. This is 1 for high block, 0 for low block.
    unsigned int address_to_print_complete;
    unsigned char address_to_print_hi;
    unsigned char address_to_print_lo;
    

    printf("\r\nBLOCK READ START ADDR (000000 - 01F3FF):\n\r");
    user_internal_address_complete = Get6HexDigits(0);

    if(user_internal_address_complete > 0x1f3ff){
        printf("\r\nInvalid address, default address used: 000000\n\r");
        user_internal_address_complete = 0;
    }

    printf("\r\nBLOCK SIZE (betweeen 000001 and 01F400):\n\r");
    user_block_size = Get6HexDigits(0);

    if (user_block_size > I2C_EEPROM_BLOCK_SIZE_LIMIT || user_block_size ==0)
    {
        printf("\r\nSize entered invalid, size set to 0000ff)\n\r");
        user_block_size = 0xff;
    }

    if(user_internal_address_complete > 0xF9FF){
        // we want to access the higher block (set block bit to 1)
        user_internal_address_complete = user_internal_address_complete - 0xFA00;
        internal_address_hi = user_internal_address_complete >> 8;
        internal_address_lo = user_internal_address_complete;
        control_byte = I2C_EEPROM_CTRLBYTE_HI_BLOCK_WRITE; // write ebecause we need to write the address first
        block_flag = 1;
    }
    else{
        // we want to acess the lower block (set block bit to 0) 
        internal_address_hi = user_internal_address_complete >> 8;
        internal_address_lo = user_internal_address_complete;
        control_byte = I2C_EEPROM_CTRLBYTE_LO_BLOCK_WRITE; // write because we need to write the address first
        block_flag = 0;
    }

    // check that device is ready
    I2C_wait_transmit_finish();
    // write the slave address
    I2C_CORE_TRANSMIT = control_byte; // this contains the slave address,
    I2C_CORE_COMMAND = I2C_SlaveWriteStart;
    I2C_wait_transmit_finish();
    I2C_check_slave_acknowledge();

    I2C_CORE_TRANSMIT = internal_address_hi;
    I2C_CORE_COMMAND = I2C_SlaveWrite;
    I2C_wait_transmit_finish();
    I2C_check_slave_acknowledge();

    I2C_CORE_TRANSMIT = internal_address_lo;
    I2C_CORE_COMMAND = I2C_SlaveWrite;
    I2C_wait_transmit_finish();
    I2C_check_slave_acknowledge();

    // control byte still contains the write bit 
    I2C_CORE_TRANSMIT = control_byte | 0x01;
    I2C_CORE_COMMAND = I2C_SlaveWriteStart;
    I2C_wait_transmit_finish();
    I2C_check_slave_acknowledge();

    next_address_tracker = internal_address_lo | (internal_address_hi << 8); // next_address_tracker now contains the starting hex address within the block
    next_address_tracker++;   

    // from here we can start looping

    for (total_bytes_tracker = 1; total_bytes_tracker < user_block_size; total_bytes_tracker++){ // this runs from 1 to 127
        
        change_block_flag = 0; // resets the flag used 

        if (block_flag){
            address_to_print_complete = next_address_tracker - 1 +  0xFA00;

        }
        else {
            address_to_print_complete = next_address_tracker - 1;
        }

        address_to_print_hi = address_to_print_complete >> 8;
        address_to_print_lo = address_to_print_complete;
        
        //printf("\r\nnext_address_tracker = %x\n", next_address_tracker); // remove later, just for debug purposes
        if(next_address_tracker > 0xF9FF){// if we are exceeding the block size, we should switch blocks
            // we need to change blocks next time
            I2C_CORE_COMMAND = I2C_SlaveRead; // send the current byte but stop the transaction
            I2C_wait_transmit_finish();
            while(I2C_CORE_STATUS & 0x01 == 0x00);// will stay here unless the core status bit 0 is set to 1

            read_byte = I2C_CORE_RECEIVE;
            I2C_CORE_COMMAND = I2C_SlaveStop; // instead of sending a stop command
            
            change_block_flag = 1;
            printf("\r\nADDR: %x%x, DATA: %x\r\n",address_to_print_hi, address_to_print_lo,read_byte);
        }        
        else {    
            I2C_CORE_COMMAND = I2C_SlaveReadAck;
            I2C_wait_transmit_finish();
            // poll status to know when we can read from receive register
            while(I2C_CORE_STATUS & 0x01 == 0x00);// will stay here unless the core status bit 0 is set to 1

            read_byte = I2C_CORE_RECEIVE;
            
            printf("\r\nADDR: %x%x, DATA: %x\r\n",address_to_print_hi, address_to_print_lo,read_byte);
        }

        if(change_block_flag == 1){// if we are exceeding the block size, we should switch blocks
            // we need to change blocks
            // setup for the batch of writes
            // need to determine this control_byte again (could just not the block bit)
        
            control_byte = control_byte ^= I2C_EEPROM_BLOCK_FLIPPER;
            I2C_wait_transmit_finish();
            I2C_CORE_TRANSMIT = control_byte;
            I2C_CORE_COMMAND = I2C_SlaveWriteStart;
            I2C_wait_transmit_finish();
            I2C_check_slave_acknowledge();

            // need to determine the internal address of the new block
            // if we changed block, it should be 0000 in the new block
            internal_address_hi = 0x00;
            internal_address_lo = 0x00;

            I2C_CORE_TRANSMIT = internal_address_hi;
            I2C_CORE_COMMAND = I2C_SlaveWrite;
            I2C_wait_transmit_finish();
            I2C_check_slave_acknowledge();

            I2C_CORE_TRANSMIT = internal_address_lo;
            I2C_CORE_COMMAND = I2C_SlaveWrite;
            I2C_wait_transmit_finish();
            I2C_check_slave_acknowledge();

            I2C_CORE_TRANSMIT = control_byte | 0x01;
            I2C_CORE_COMMAND = I2C_SlaveWriteStart;
            I2C_wait_transmit_finish();
            I2C_check_slave_acknowledge();

            //reset the address tracker
            next_address_tracker = 0x00;

            block_flag = !block_flag;

            // //reset the flag
            // change_block_flag = 0;
        }

        next_address_tracker++;
        //bytes_counter++; // counts the number of bytes sent (up to 128)
    }

    
    if (block_flag){
        address_to_print_complete = next_address_tracker - 1 +  0xFA00;
    }
    else {
        address_to_print_complete = next_address_tracker - 1;
    }

    address_to_print_hi = address_to_print_complete >> 8;
    address_to_print_lo = address_to_print_complete;

    I2C_CORE_COMMAND = I2C_SlaveRead;
    I2C_wait_transmit_finish();
    // poll status to know when we can read from receive register
    while(I2C_CORE_STATUS & 0x01 == 0x00);// will stay here unless the core status bit 0 is set to 1
    read_byte = I2C_CORE_RECEIVE;
    I2C_CORE_COMMAND = I2C_SlaveStop;// this also rests the flag above
    printf("\r\nADDR: %x%x, DATA: %x\r\n",address_to_print_hi, address_to_print_lo,read_byte);
    printf("\r\nBlock Read operation complete\r\n");
}

//****************************************************************
// 4. A function to Write a specified sized BLOCK of data up to 128 k bytes starting at any address in the EEPROM chip. Use for example something like page write mode. For testing, use incrementing 8 bit values as your data.
void I2C_EEPROM_BLOCKWRITE(void){

    unsigned char control_byte;
    unsigned char byte_to_write;
    unsigned int user_internal_address_complete;
    unsigned int user_block_size;                   // this would hold up to 1F400
    unsigned char internal_address_hi;
    unsigned char internal_address_lo;
    unsigned int next_address_tracker;                   // keeps track of the internal address so that we can know when to switch the blocks
    unsigned int total_bytes_tracker;               // keeps track of how close we are to the user_block_size
    unsigned int bytes_counter;                     // keeps track of when the 128 bytes have been written over so the current read can be ended and a new one started
    unsigned int change_block_flag;                 // flag used to know when to send out the new control and address

    
    printf("\r\nENTER BLOCK WRITE ADDR (000000 - 01F3FF)\n\r");
    user_internal_address_complete = Get6HexDigits(0);

    if(user_internal_address_complete > 0x1f3ff){
        printf("\r\nInvalid address, default address used: 000000\n\r");
        user_internal_address_complete = 0;
    }

    printf("\r\nENTER BLOCK SIZE (000001 - 01F400)\n\r");
    user_block_size = Get6HexDigits(0);

    if (user_block_size > I2C_EEPROM_BLOCK_SIZE_LIMIT || user_block_size ==0)
    {
        printf("\r\nSize entered invalid, size set to HEX ff)\n\r");
        user_block_size = 0xff;
    }

    if(user_internal_address_complete > 0xF9FF){
        // we want to access the higher block (set block bit to 1)
        user_internal_address_complete = user_internal_address_complete - 0xFA00;
        internal_address_hi = user_internal_address_complete >> 8;
        internal_address_lo = user_internal_address_complete;
        control_byte = I2C_EEPROM_CTRLBYTE_HI_BLOCK_WRITE; // write ebecause we need to write the address first
    }
    else{
        // we want to acess the lower block (set block bit to 0) 
        internal_address_hi = user_internal_address_complete >> 8;
        internal_address_lo = user_internal_address_complete;
        control_byte = I2C_EEPROM_CTRLBYTE_LO_BLOCK_WRITE; // write because we need to write the address first
    }

    //printf("\r\nHi address byte: %x\nlo address byte: %x\n", internal_address_hi, internal_address_lo); // remove later, just for debug purposes

    // give an initial value to be written:
    printf("\r\nEnter HEX byte data to write\n\r");
    byte_to_write = Get2HexDigits(0);
    
    // setup for the batch of writes
    I2C_wait_transmit_finish();
    I2C_CORE_TRANSMIT = control_byte;
    I2C_CORE_COMMAND = I2C_SlaveWriteStart;
    I2C_wait_transmit_finish();
    I2C_check_slave_acknowledge();

    I2C_CORE_TRANSMIT = internal_address_hi;
    I2C_CORE_COMMAND = I2C_SlaveWrite;
    I2C_wait_transmit_finish();
    I2C_check_slave_acknowledge();

    I2C_CORE_TRANSMIT = internal_address_lo;
    I2C_CORE_COMMAND = I2C_SlaveWrite;
    I2C_wait_transmit_finish();
    I2C_check_slave_acknowledge();

    // address tracker initialized:
    next_address_tracker = internal_address_lo | (internal_address_hi << 8); // next_address_tracker now contains the starting hex address within the block
    next_address_tracker++;                           // next_address_tracker now contains the next address to write data to
    bytes_counter = 0;                                                       // keeps track of the bytes count up to 128

    // enter loop to keep sending data until the 127th byte is sent out
    // the 128th byte is to be sent with the stop
    for (total_bytes_tracker = 1; total_bytes_tracker < user_block_size; total_bytes_tracker++){ // this runs from 1 to 127
        I2C_CORE_TRANSMIT = byte_to_write;
        // try resetting the change_block_flag here:
        change_block_flag = 0;
        //printf("\r\nnext_address_tracker = %x\n", next_address_tracker); // remove later, just for debug purposes
        if(next_address_tracker > 0xF9FF){// if we are exceeding the block size, we should switch blocks
            // we need to change blocks next time
            I2C_CORE_COMMAND = I2C_SlaveWriteStop; // send the current byte but stop the transaction
            I2C_wait_transmit_finish();
            I2C_check_slave_acknowledge();
            change_block_flag = 1;
            printf("\r\nChanging Block within EEPROM\n\r");
        }        
        else {
            I2C_CORE_COMMAND = I2C_SlaveWrite; // these don't send a stop 
            I2C_wait_transmit_finish();
            I2C_check_slave_acknowledge();

            // printf("\r\nWrote byte using Page Write\n\r");
        }

        if(change_block_flag == 1){// if we are exceeding the block size, we should switch blocks
            // we need to change blocks
            // setup for the batch of writes
            // need to determine this control_byte again (could just not the block bit)
        
            control_byte = control_byte ^= I2C_EEPROM_BLOCK_FLIPPER;
            I2C_wait_transmit_finish();
            I2C_CORE_TRANSMIT = control_byte;
            I2C_CORE_COMMAND = I2C_SlaveWriteStart;
            I2C_wait_transmit_finish();
            I2C_check_slave_acknowledge();

            // need to determine the internal address of the new block
            // if we changed block, it should be 0000 in the new block
            internal_address_hi = 0x00;
            internal_address_lo = 0x00;

            I2C_CORE_TRANSMIT = internal_address_hi;
            I2C_CORE_COMMAND = I2C_SlaveWrite;
            I2C_wait_transmit_finish();
            I2C_check_slave_acknowledge();

            I2C_CORE_TRANSMIT = internal_address_lo;
            I2C_CORE_COMMAND = I2C_SlaveWrite;
            I2C_wait_transmit_finish();
            I2C_check_slave_acknowledge();

            //reset the address tracker
            next_address_tracker = 0x01;

            // //reset the flag
            // change_block_flag = 0;
        }

        if(bytes_counter == 127){ // already sent 127 bytes, this 128th byte is to be sent with a stop, then set the write again
            if(change_block_flag ==0){
                I2C_CORE_COMMAND = I2C_SlaveWriteStop; // we send a stop since we need to get set the write again
                I2C_wait_transmit_finish();
                I2C_check_slave_acknowledge();

                // send the same control_byte again, send the internal address again.
                // setup for the next batch of writes
                printf("\r\nSent 128 bytes in a row, restarting setup\n\r");
                I2C_wait_transmit_finish();
                I2C_CORE_TRANSMIT = control_byte;
                I2C_CORE_COMMAND = I2C_SlaveWriteStart;
                I2C_wait_transmit_finish();
                I2C_check_slave_acknowledge();

                internal_address_lo = next_address_tracker;
                internal_address_hi = next_address_tracker >> 8;

                I2C_CORE_TRANSMIT = internal_address_hi;
                I2C_CORE_COMMAND = I2C_SlaveWrite;
                I2C_wait_transmit_finish();
                I2C_check_slave_acknowledge();

                I2C_CORE_TRANSMIT = internal_address_lo;
                I2C_CORE_COMMAND = I2C_SlaveWrite;
                I2C_wait_transmit_finish();
                I2C_check_slave_acknowledge();
            }
            // printf("\r\nSent 128 bytes, setup the write again \n\r");
            //rset the bytes_counter
            bytes_counter = 0;
        }
        next_address_tracker++;
        bytes_counter++; // counts the number of bytes sent (up to 128)
    }
    // send the last byte of the user defined block
    I2C_CORE_TRANSMIT = byte_to_write;
    I2C_CORE_COMMAND = I2C_SlaveWriteStop;
    I2C_wait_transmit_finish();
    I2C_check_slave_acknowledge();

    printf("\r\nLast byte write complete. Check results\n\r");
}
//////////////////////////////////////////////////////////////////////////////////////////////////

void PCF8591_Gen(void){
    unsigned char amplitude;
    int  period = 0;
    int dirFlag = 0;
    int timer = 0;
    int i = 1;
    int ledValue = 0;
    int counter = 1;
    printf("\r\nGenerating a signal input an amplitude DAC (00-FF in HEX)\r\n");

    // printf("%.6f",0.001023);
    amplitude = Get2HexDigits(0);

    I2C_wait_transmit_finish();

    printf("\r\nAmplitude input is: %d\r\n", amplitude);

    I2C_CORE_TRANSMIT = ADC_SLAVE_ADDRESS_MWRITE; //write initial address
    I2C_CORE_COMMAND = I2C_SlaveWriteStart;

    I2C_wait_transmit_finish();
    I2C_check_slave_acknowledge();

    // printf("Sent slave address \r\n");

    I2C_CORE_TRANSMIT = ADC_ANALOG_OUT_CTRL_BYTE;
    I2C_CORE_COMMAND = I2C_SlaveWrite;

    I2C_wait_transmit_finish();
    I2C_check_slave_acknowledge();

    // printf("Sent ctrl byte \r\n");
    I2C_CORE_TRANSMIT = amplitude;
    I2C_CORE_COMMAND = I2C_SlaveWrite;
    I2C_wait_transmit_finish();
    I2C_check_slave_acknowledge();

    printf("Stop wave press key0\r\n");

    period = (int)(256-amplitude);
    while(1)
    {
        for(i=0;i<((period/50)+1)*10;i++){
            // printf("i = %d",i);
            if(i==(((period/50)+1)*10-1)){
                if (dirFlag==0){
                // printf("amplitude is %d\r\n",(int)amplitude);
                    if((int)counter >= (int)amplitude)
                    {
                        dirFlag = 1;
                    }
                    else
                    {
                        counter++;
                    }
                }
                else{
                    if(counter <= 0)
                    {
                        dirFlag = 0;
                    }
                    else
                    {
                    counter--;
                    }
                }
                i=0;
            }
        // printf("counter value is = %d\r\n",counter);
        I2C_CORE_TRANSMIT = counter;
        I2C_CORE_COMMAND = I2C_SlaveWrite;
        I2C_wait_transmit_finish();
        I2C_check_slave_acknowledge();
        }

    }


}

void PCF8591_Read(void){
    unsigned char read_byte;
    int i =0;
    I2C_wait_transmit_finish();

    printf("\r\nReading ADC channels\r\n");

    I2C_CORE_TRANSMIT = ADC_SLAVE_ADDRESS_MWRITE; //write initial address
    I2C_CORE_COMMAND = I2C_SlaveWriteStart;

    I2C_wait_transmit_finish();
    I2C_check_slave_acknowledge();

    I2C_CORE_TRANSMIT = ADC_ANALOG_READ_CTRL_BYTE;
    I2C_CORE_COMMAND = I2C_SlaveWrite;

    I2C_wait_transmit_finish();
    I2C_check_slave_acknowledge();

    I2C_CORE_TRANSMIT = ADC_SLAVE_ADDRESS_MREAD; //write initial address
    I2C_CORE_COMMAND = I2C_SlaveWriteStart;

    I2C_wait_transmit_finish();
    I2C_check_slave_acknowledge();

    for (i =0; i<4;i++){

        if (i == 3){
            I2C_CORE_COMMAND = I2C_SlaveRead;
        }
        else{
            I2C_CORE_COMMAND = I2C_SlaveReadAck;
        }        
        I2C_wait_transmit_finish();

        while(I2C_CORE_STATUS & 0x01 == 0x00);// will stay here unless the core status bit 0 is set to 1
        //printf("Getting byte \r\n");
        read_byte = I2C_CORE_RECEIVE;
        if(i == 1){
            printf("\r\nThermistor read is: %d\r\n", read_byte);
        }
        else if(i == 2){
            printf("\r\nPotentiometer read is: %d\r\n", read_byte);
        }
        else if (i==3){
            printf("\r\nPhotoresistor read is: %d\r\n", read_byte);
        }
        // else 
        //     printf("Garbage value is: %d\r\n",read_byte);
    }
    I2C_CORE_COMMAND = I2C_SlaveStop;
    I2C_wait_transmit_finish();


}


//////////////////////////////////////////////////////////////////////////////////////////////////
// IMPORTANT
// TG68 does not support the Native Trace mode of the original 68000 so tracing
// has to be done with an interrupt (IRQ Level 6)
//
// To allow the 68000 to execute one more instruction after each pseudo trace (IRQ6)
// the IRQ is removed in hardware once the TG68 reads the IRQ autovector (i.e. acknowledges the IRQ)
//
// on return from the IRQ service handler, the first access to the user memory program space
// generates a fresh IRQ (in hardware) to generate a new trace, this allows the tg68 to
// execute one more new instruction (without it the TG68 would trace on the same instruction
// each time and not after the next one). It also means it doesn't simgle step outside the user
// program area
//
// The bottom line is the Trace handler, which includes the Dump registers below
// cannot access the user memory to display for example the Instruction Opcode or to disassemble etc
// as this would lead to a new IRQ being reset and the TG68 would trace on same instruction
// NOT SURE THIS IS TRUE NOW THAT TRACE HANDLER HAS BEEN MODIVIED TO NOT AUTOMATICALLY GENERATE A TRACE EXCEPTION
// INSTEAD IT IS DONE IN THE 'N' COMMAND FOR NEXT
/////////////////////////////////////////////////////////////////////////////////////////////////////


void DumpRegisters()
{
    short i, x, j, k ;
    unsigned char c, *BytePointer;

// buld up strings for displaying watchpoints

    for(x = 0; x < (short)(8); x++)
    {
        if(WatchPointSetOrCleared[x] == 1)
        {
            sprintf(WatchPointString[x], "$%08X  ", WatchPointAddress[x]) ;
            BytePointer = (char *)(WatchPointAddress[x]) ;

            for(j = 0; j < (short)(16); j+=2)
            {
                for(k = 0; k < (short)(2); k++)
                {
                    sprintf(TempString, "%02X", BytePointer[j+k]) ;
                    strcat(WatchPointString[x], TempString) ;
                }
                strcat(WatchPointString[x]," ") ;
            }

            strcat(WatchPointString[x], "  ") ;
            BytePointer = (char *)(WatchPointAddress[x]) ;

            for(j = 0; j < (short)(16); j++)
            {
                c = ((char)(BytePointer[j]) & 0x7f) ;
                if((c > (char)(0x7f)) || (c < (char)(' ')))
                    sprintf(TempString, ".") ;
                else
                    sprintf(TempString, "%c", BytePointer[j]) ;
                strcat(WatchPointString[x], TempString) ;
            }
        }
        else
            strcpy(WatchPointString[x], "") ;
    }

    printf("\r\n\r\n D0 = $%08X  A0 = $%08X",d0,a0) ;
    printf("\r\n D1 = $%08X  A1 = $%08X",d1,a1) ;
    printf("\r\n D2 = $%08X  A2 = $%08X",d2,a2) ;
    printf("\r\n D3 = $%08X  A3 = $%08X",d3,a3) ;
    printf("\r\n D4 = $%08X  A4 = $%08X",d4,a4) ;
    printf("\r\n D5 = $%08X  A5 = $%08X",d5,a5) ;
    printf("\r\n D6 = $%08X  A6 = $%08X",d6,a6) ;
    printf("\r\n D7 = $%08X  A7 = $%08X",d7,((SR & (unsigned short int)(0x2000)) == ((unsigned short int)(0x2000))) ? SSP : USP) ;
    printf("\r\n\r\nUSP = $%08X  (A7) User SP", USP ) ;
    printf("\r\nSSP = $%08X  (A7) Supervisor SP", SSP) ;
    printf("\r\n SR = $%04X   ",SR) ;

// display the status word in characters etc.

    printf("   [") ;
    if((SR & (unsigned short int)(0x8000)) == (unsigned short int)(0x8000)) putchar('T') ; else putchar('-') ;      // Trace bit(bit 15)
    if((SR & (unsigned short int)(0x2000)) == (unsigned short int)(0x2000)) putchar('S') ; else putchar('U') ;      // supervisor bit  (bit 13)

    if((SR & (unsigned short int)(0x0400)) == (unsigned short int)(0x0400)) putchar('1') ; else putchar('0') ;      // IRQ2 Bit (bit 10)
    if((SR & (unsigned short int)(0x0200)) == (unsigned short int)(0x0200)) putchar('1') ; else putchar('0') ;      // IRQ1 Bit (bit 9)
    if((SR & (unsigned short int)(0x0100)) == (unsigned short int)(0x0100)) putchar('1') ; else putchar('0') ;      // IRQ0 Bit (bit 8)

    if((SR & (unsigned short int)(0x0010)) == (unsigned short int)(0x0010)) putchar('X') ; else putchar('-') ;      // X Bit (bit 4)
    if((SR & (unsigned short int)(0x0008)) == (unsigned short int)(0x0008)) putchar('N') ; else putchar('-') ;      // N Bit (bit 3)
    if((SR & (unsigned short int)(0x0004)) == (unsigned short int)(0x0004)) putchar('Z') ; else putchar('-') ;      // Z Bit (bit 2)
    if((SR & (unsigned short int)(0x0002)) == (unsigned short int)(0x0002)) putchar('V') ; else putchar('-') ;      // V Bit (bit 1)
    if((SR & (unsigned short int)(0x0001)) == (unsigned short int)(0x0001)) putchar('C') ; else putchar('-') ;      // C Bit (bit 0)
    putchar(']') ;

    printf("\r\n PC = $%08X  ", PC) ;
    if(*(unsigned short int *)(PC) == 0x4e4e)
        printf("[@ BREAKPOINT]") ;

    printf("\r\n") ;

    for(i=0; i < 8; i++)    {
        if(WatchPointSetOrCleared[i] == 1)
            printf("\r\nWP%d = %s", i, WatchPointString[i]) ;
    }

}

// Trace Exception Handler
void DumpRegistersandPause(void)
{
    printf("\r\n\r\n\r\n\r\n\r\n\r\nSingle Step  :[ON]") ;
    printf("\r\nBreak Points :[Disabled]") ;
    DumpRegisters() ;
    printf("\r\nPress <SPACE> to Execute Next Instruction");
    printf("\r\nPress <ESC> to Resume Program") ;
    menu() ;
}

void ChangeRegisters(void)
{
    // get register name d0-d7, a0-a7, up, sp, sr, pc

    int reg_val ;
    char c, reg[3] ;

    reg[0] = tolower(_getch()) ;
    reg[1] = c = tolower(_getch()) ;

    if(reg[0] == (char)('d'))  {    // change data register
        if((reg[1] > (char)('7')) || (reg[1] < (char)('0'))) {
            printf("\r\nIllegal Data Register : Use D0-D7.....\r\n") ;
            return ;
        }
        else {
            printf("\r\nD%c = ", c) ;
            reg_val = Get8HexDigits(0) ;    // read 32 bit value from user keyboard
        }

        // bit cludgy but d0-d7 not stored as an array for good reason
        if(c == (char)('0'))
            d0 = reg_val ;
        else if(c == (char)('1'))
            d1 = reg_val ;
        else if(c == (char)('2'))
            d2 = reg_val ;
        else if(c == (char)('3'))
            d3 = reg_val ;
        else if(c == (char)('4'))
            d4 = reg_val ;
        else if(c == (char)('5'))
            d5 = reg_val ;
        else if(c == (char)('6'))
            d6 = reg_val ;
        else
            d7 = reg_val ;
    }
    else if(reg[0] == (char)('a'))  {    // change address register, a7 is the user stack pointer, sp is the system stack pointer
        if((c > (char)('7')) || (c < (char)('0'))) {
            printf("\r\nIllegal Address Register : Use A0-A7.....\r\n") ;
            return ;
        }
        else {
            printf("\r\nA%c = ", c) ;
            reg_val = Get8HexDigits(0) ;    // read 32 bit value from user keyboard
        }
        // bit cludgy but a0-a7 not stored as an array for good reason
        if(c == (char)('0'))
            a0 = reg_val ;
        else if(c == (char)('1'))
            a1 = reg_val ;
        else if(c == (char)('2'))
            a2 = reg_val ;
        else if(c == (char)('3'))
            a3 = reg_val ;
        else if(c == (char)('4'))
            a4 = reg_val ;
        else if(c == (char)('5'))
            a5 = reg_val ;
        else if(c == (char)('6'))
            a6 = reg_val ;
        else
            USP = reg_val ;
    }
    else if((reg[0] == (char)('u')) && (c == (char)('s')))  {
           if(tolower(_getch()) == 'p')  {    // change user stack pointer
                printf("\r\nUser SP = ") ;
                USP = Get8HexDigits(0) ;    // read 32 bit value from user keyboard
           }
           else {
                printf("\r\nIllegal Register....") ;
                return ;
           }
    }

    else if((reg[0] == (char)('s')) && (c == (char)('s')))  {
           if(tolower(_getch()) == 'p')  {    // change system stack pointer
                printf("\r\nSystem SP = ") ;
                SSP = Get8HexDigits(0) ;    // read 32 bit value from user keyboard
           }
           else {
                printf("\r\nIllegal Register....") ;
                return ;
           }
    }

    else if((reg[0] == (char)('p')) && (c == (char)('c')))  {    // change program counter
          printf("\r\nPC = ") ;
          PC = Get8HexDigits(0) ;    // read 32 bit value from user keyboard
    }

    else if((reg[0] == (char)('s')) && (c == (char)('r')))  {    // change status register
          printf("\r\nSR = ") ;
          SR = Get4HexDigits(0) ;    // read 16 bit value from user keyboard
    }
    else
        printf("\r\nIllegal Register: Use A0-A7, D0-D7, SSP, USP, PC or SR\r\n") ;

    DumpRegisters() ;
}

void BreakPointDisplay(void)
{
   int i, BreakPointsSet = 0 ;

// any break points  set

    for(i = 0; i < 8; i++)  {
       if(BreakPointSetOrCleared[i] == 1)
            BreakPointsSet = 1;
    }

    if(BreakPointsSet == 1) {
        printf("\r\n\r\nNum     Address      Instruction") ;
        printf("\r\n---     ---------    -----------") ;
    }
    else
        printf("\r\nNo BreakPoints Set") ;


    for(i = 0; i < 8; i++)  {
    // put opcode back, then put break point back
        if(BreakPointSetOrCleared[i] == 1)  {
            *(unsigned short int *)(BreakPointAddress[i]) = BreakPointInstruction[i];
            *(unsigned short int *)(BreakPointAddress[i]) = (unsigned short int)(0x4e4e) ;
            printf("\r\n%3d     $%08x",i, BreakPointAddress[i]) ;
        }
    }
    printf("\r\n") ;
}

void WatchPointDisplay(void)
{
   int i ;
   int WatchPointsSet = 0 ;

// any watchpoints set

    for(i = 0; i < 8; i++)  {
       if(WatchPointSetOrCleared[i] == 1)
            WatchPointsSet = 1;
    }

    if(WatchPointsSet == 1) {
        printf("\r\nNum     Address") ;
        printf("\r\n---     ---------") ;
    }
    else
        printf("\r\nNo WatchPoints Set") ;

    for(i = 0; i < 8; i++)  {
        if(WatchPointSetOrCleared[i] == 1)
            printf("\r\n%3d     $%08x",i, WatchPointAddress[i]) ;
     }
    printf("\r\n") ;
}

void BreakPointClear(void)
{
    unsigned int i ;
    volatile unsigned short int *ProgramBreakPointAddress ;

    BreakPointDisplay() ;

    printf("\r\nEnter Break Point Number: ") ;
    i = xtod(_getch()) ;           // get break pointer number

    if((i < 0) || (i > 7))   {
        printf("\r\nIllegal Range : Use 0 - 7") ;
        return ;
    }

    if(BreakPointSetOrCleared[i] == 1)  {       // if break point set
        ProgramBreakPointAddress = (volatile unsigned short int *)(BreakPointAddress[i]) ;     // point to the instruction in the user program we are about to change
        BreakPointAddress[i] = 0 ;
        BreakPointSetOrCleared[i] = 0 ;
        *ProgramBreakPointAddress = BreakPointInstruction[i] ;  // put original instruction back
        BreakPointInstruction[i] = 0 ;
        printf("\r\nBreak Point Cleared.....\r\n") ;
    }
    else
        printf("\r\nBreak Point wasn't Set.....") ;

    BreakPointDisplay() ;
    return ;
}

void WatchPointClear(void)
{
    unsigned int i ;

    WatchPointDisplay() ;

    printf("\r\nEnter Watch Point Number: ") ;
    i = xtod(_getch()) ;           // get watch pointer number

    if((i < 0) || (i > 7))   {
        printf("\r\nIllegal Range : Use 0 - 7") ;
        return ;
    }

    if(WatchPointSetOrCleared[i] == 1)  {       // if watch point set
        WatchPointAddress[i] = 0 ;
        WatchPointSetOrCleared[i] = 0 ;
        printf("\r\nWatch Point Cleared.....\r\n") ;
    }
    else
        printf("\r\nWatch Point Was not Set.....") ;

    WatchPointDisplay() ;
    return ;

}

void DisableBreakPoints(void)
{
   int i ;
   volatile unsigned short int *ProgramBreakPointAddress ;

   for(i = 0; i < 8; i++)  {
      if(BreakPointSetOrCleared[i] == 1)    {                                                    // if break point set
          ProgramBreakPointAddress = (volatile unsigned short int *)(BreakPointAddress[i]) ;     // point to the instruction in the user program where the break point has been set
          *ProgramBreakPointAddress = BreakPointInstruction[i];                                  // copy the instruction back to the user program overwritting the $4e4e
      }
   }
}

void EnableBreakPoints(void)
{
   int i ;
   volatile unsigned short int *ProgramBreakPointAddress ;

   for(i = 0; i < 8; i++)  {
      if(BreakPointSetOrCleared[i] == 1)    {                                                     // if break point set
           ProgramBreakPointAddress = (volatile unsigned short int *)(BreakPointAddress[i]) ;     // point to the instruction in the user program where the break point has been set
           *ProgramBreakPointAddress = (unsigned short int)(0x4e4e);                              // put the breakpoint back in user program
      }
   }
}

void KillAllBreakPoints(void)
{
   int i ;
   volatile unsigned short int *ProgramBreakPointAddress ;

   for(i = 0; i < 8; i++)  {
       // clear BP
       ProgramBreakPointAddress = (volatile unsigned short int *)(BreakPointAddress[i]) ;     // point to the instruction in the user program where the break point has been set
       *ProgramBreakPointAddress = BreakPointInstruction[i];                                  // copy the instruction back to the user program
       BreakPointAddress[i] = 0 ;                                                             // set BP address to NULL
       BreakPointInstruction[i] = 0 ;
       BreakPointSetOrCleared[i] = 0 ;                                                        // mark break point as cleared for future setting
   }
   //BreakPointDisplay() ;       // display the break points
}

void KillAllWatchPoints(void)
{
   int i ;

   for(i = 0; i < 8; i++)  {
       WatchPointAddress[i] = 0 ;                                                             // set BP address to NULL
       WatchPointSetOrCleared[i] = 0 ;                                                        // mark break point as cleared for future setting
   }
   //WatchPointDisplay() ;       // display the break points
}


void SetBreakPoint(void)
{
    int i ;
    int BPNumber;
    int BPAddress;
    volatile unsigned short int *ProgramBreakPointAddress ;

    // see if any free break points

    for(i = 0; i < 8; i ++) {
        if( BreakPointSetOrCleared[i] == 0)
            break ;         // if spare BP found allow user to set it
    }

    if(i == 8) {
        printf("\r\nNo FREE Break Points.....") ;
        return ;
    }

    printf("\r\nBreak Point Address: ") ;
    BPAddress = Get8HexDigits(0) ;
    ProgramBreakPointAddress = (volatile unsigned short int *)(BPAddress) ;     // point to the instruction in the user program we are about to change

    if((BPAddress & 0x00000001) == 0x00000001)  {   // cannot set BP at an odd address
        printf("\r\nError : Break Points CANNOT be set at ODD addresses") ;
        return ;
    }

    if(BPAddress < 0x00008000)  {   // cannot set BP in ROM
        printf("\r\nError : Break Points CANNOT be set for ROM in Range : [$0-$00007FFF]") ;
        return ;
    }

    // search for first free bp or existing same BP

    for(i = 0; i < 8; i++)  {
        if(BreakPointAddress[i] == BPAddress)   {
            printf("\r\nError: Break Point Already Exists at Address : %08x\r\n", BPAddress) ;
            return ;
        }
        if(BreakPointSetOrCleared[i] == 0) {
            // set BP here
            BreakPointSetOrCleared[i] = 1 ;                                 // mark this breakpoint as set
            BreakPointInstruction[i] = *ProgramBreakPointAddress ;          // copy the user program instruction here so we can put it back afterwards
            printf("\r\nBreak Point Set at Address: [$%08x]", ProgramBreakPointAddress) ;
            *ProgramBreakPointAddress = (unsigned short int)(0x4e4e)    ;   // put a Trap14 instruction at the user specified address
            BreakPointAddress[i] = BPAddress ;                              // record the address of this break point in the debugger
            printf("\r\n") ;
            BreakPointDisplay() ;       // display the break points
            return ;
        }
    }
}

void SetWatchPoint(void)
{
    int i ;
    int WPNumber;
    int WPAddress;
    volatile unsigned short int *ProgramWatchPointAddress ;

    // see if any free break points

    for(i = 0; i < 8; i ++) {
        if( WatchPointSetOrCleared[i] == 0)
            break ;         // if spare WP found allow user to set it
    }

    if(i == 8) {
        printf("\r\nNo FREE Watch Points.....") ;
        return ;
    }

    printf("\r\nWatch Point Address: ") ;
    WPAddress = Get8HexDigits(0) ;

    // search for first free wp or existing same wp

    for(i = 0; i < 8; i++)  {
        if(WatchPointAddress[i] == WPAddress && WPAddress != 0)   {     //so we can set a wp at 0
            printf("\r\nError: Watch Point Already Set at Address : %08x\r\n", WPAddress) ;
            return ;
        }
        if(WatchPointSetOrCleared[i] == 0) {
            WatchPointSetOrCleared[i] = 1 ;                                 // mark this watchpoint as set
            printf("\r\nWatch Point Set at Address: [$%08x]", WPAddress) ;
            WatchPointAddress[i] = WPAddress ;                              // record the address of this watch point in the debugger
            printf("\r\n") ;
            WatchPointDisplay() ;       // display the break points
            return ;
        }
    }
}


void HandleBreakPoint(void)
{
    volatile unsigned short int *ProgramBreakPointAddress ;

    // now we have to put the break point back to run the instruction
    // PC will contain the address of the TRAP instruction but advanced by two bytes so lets play with that

    PC = PC - 2 ;  // ready for user to resume after reaching breakpoint

    printf("\r\n\r\n\r\n\r\n@BREAKPOINT") ;
    printf("\r\nSingle Step : [ON]") ;
    printf("\r\nBreakPoints : [Enabled]") ;

    // now clear the break point (put original instruction back)

    ProgramBreakPointAddress = PC ;

    for(i = 0; i < 8; i ++) {
        if(BreakPointAddress[i] == PC) {        // if we have found the breakpoint
            BreakPointAddress[i] = 0 ;
            BreakPointSetOrCleared[i] = 0 ;
            *ProgramBreakPointAddress = BreakPointInstruction[i] ;  // put original instruction back
            BreakPointInstruction[i] = 0 ;
        }
    }

    DumpRegisters() ;
    printf("\r\nPress <SPACE> to Execute Next Instruction");
    printf("\r\nPress <ESC> to Resume User Program\r\n") ;
    menu() ;
}

void UnknownCommand()
{
    printf("\r\nUnknown Command.....\r\n") ;
    Help() ;
}

// system when the users program executes a TRAP #15 instruction to halt program and return to debug monitor

void CallDebugMonitor(void)
{
    printf("\r\nProgram Ended (TRAP #15)....") ;
    menu();
}

void Breakpoint(void)
{
       char c;
       c = toupper(_getch());

        if( c == (char)('D'))                                      // BreakPoint Display
            BreakPointDisplay() ;

        else if(c == (char)('K')) {                                 // breakpoint Kill
            printf("\r\nKill All Break Points...(y/n)?") ;
            c = toupper(_getch());
            if(c == (char)('Y'))
                KillAllBreakPoints() ;
        }
        else if(c == (char)('S')) {
            SetBreakPoint() ;
        }
        else if(c == (char)('C')) {
            BreakPointClear() ;
        }
        else
            UnknownCommand() ;
}

void Watchpoint(void)
{
       char c;
       c = toupper(_getch());

        if( c == (char)('D'))                                      // WatchPoint Display
            WatchPointDisplay() ;

        else if(c == (char)('K')) {                                 // wtahcpoint Kill
            printf("\r\nKill All Watch Points...(y/n)?") ;
            c = toupper(_getch());
            if(c == (char)('Y'))
                KillAllWatchPoints() ;
        }
        else if(c == (char)('S')) {
            SetWatchPoint() ;
        }
        else if(c == (char)('C')) {
            WatchPointClear() ;
        }
        else
            UnknownCommand() ;
}



void Help(void)
{
    char *banner = "\r\n----------------------------------------------------------------" ;

    printf(banner) ;
    printf("\r\n  Debugger Command Summary") ;
    printf(banner) ;
    printf("\r\n  .(reg)       - Change Registers: e.g A0-A7,D0-D7,PC,SSP,USP,SR");
    printf("\r\n  BD/BS/BC/BK  - Break Point: Display/Set/Clear/Kill") ;
    printf("\r\n  C            - Copy Program from Flash to Main Memory") ;
    printf("\r\n  D            - Dump Memory Contents to Screen") ;
    printf("\r\n  E            - Enter String into Memory") ;
    printf("\r\n  F            - Fill Memory with Data") ;
    printf("\r\n  G            - Go Program Starting at Address: $%08X", PC) ;
    printf("\r\n  L            - Load Program (.HEX file) from Laptop") ;
    printf("\r\n  M            - Memory Examine and Change");
    printf("\r\n  P            - Program Flash Memory with User Program") ;
    printf("\r\n  R            - Display 68000 Registers") ;
    printf("\r\n  S            - Toggle ON/OFF Single Step Mode") ;
    printf("\r\n  TM           - Test Memory") ;
    printf("\r\n  TS           - Test Switches: SW7-0") ;
    printf("\r\n  TD           - Test Displays: LEDs and 7-Segment") ;
    printf("\r\n  WD/WS/WC/WK  - Watch Point: Display/Set/Clear/Kill") ;
    printf("\r\n  IR           - I2C EEPROM random read");
    printf("\r\n  IW           - I2C EEPROM random write");
    printf("\r\n  I1           - I2C EEPROM BLOCK write");
    printf("\r\n  I2           - I2C EEPROM BLOCK write");
    printf("\r\n  IG           - I2C EEPROM ADC GENERATOR");
    printf("\r\n  IC           - I2C EEPROM ADC READ");
    printf(banner) ;
}


void menu(void)
{
    char c,c1 ;

    while(1)    {
        FlushKeyboard() ;               // dump unread characters from keyboard
        printf("\r\n#") ;
        c = toupper(_getch());

        if( c == (char)('L'))                  // load s record file
             Load_SRecordFile() ;

        else if( c == (char)('D'))             // dump memory
            DumpMemory() ;

        else if( c == (char)('E'))             // Enter String into memory
            EnterString() ;

        else if( c == (char)('F'))             // fill memory
            FillMemory() ;

        else if( c == (char)('G'))  {           // go user program
            printf("\r\nProgram Running.....") ;
            printf("\r\nPress <RESET> button <Key0> on DE1 to stop") ;
            GoFlag = 1 ;
            go() ;
        }

        else if( c == (char)('M'))           // memory examine and modify
             MemoryChange() ;

        else if( c == (char)('P'))            // Program Flash Chip
             ProgramFlashChip() ;

        else if( c == (char)('C'))             // copy flash chip to ram and go
             LoadFromFlashChip();

        else if( c == (char)('R'))             // dump registers
             DumpRegisters() ;

        else if( c == (char)('.'))           // change registers
             ChangeRegisters() ;

        else if( c == (char)('B'))              // breakpoint command
            Breakpoint() ;

        else if( c == (char)('T'))  {          // Test command
             c1 = toupper(_getch()) ;
            if(c1 == (char)('M'))                    // memory test
            MemoryTest() ;
            else if( c1 == (char)('S'))              // Switch Test command
            SwitchTest() ;
            else if( c1 == (char)('D'))              // display Test command
            TestLEDS() ;
            else if (c1 == (char)('G'))
                PCF8591_Gen();
            else if (c1 == (char)('C'))
                PCF8591_Read();
             else
                UnknownCommand() ;
        }

        else if( c == (char)(' ')) {             // Next instruction command
            DisableBreakPoints() ;
            if(Trace == 1 && GoFlag == 1)   {    // if the program is running and trace mode on then 'N' is valid
                TraceException = 1 ;             // generate a trace exception for the next instruction if user wants to single step though next instruction
                return ;
            }
            else
                printf("\r\nError: Press 'G' first to start program") ;
        }

        else if( c == (char)('S')) {             // single step
             if(Trace == 0) {
                DisableBreakPoints() ;
                printf("\r\nSingle Step  :[ON]") ;
                printf("\r\nBreak Points :[Disabled]") ;
                SR = SR | (unsigned short int)(0x8000) ;    // set T bit in status register
                printf("\r\nPress 'G' to Trace Program from address $%X.....",PC) ;
                printf("\r\nPush <RESET Button> to Stop.....") ;
                DumpRegisters() ;

                Trace = 1;
                TraceException = 1;
                x = *(unsigned int *)(0x00000074) ;       // simulate responding to a Level 5 IRQ by reading vector to reset Trace exception generator
            }
            else {
                Trace = 0 ;
                TraceException = 0 ;
                x = *(unsigned int *)(0x00000074) ;       // simulate responding to a Level 5 IRQ by reading vector to reset Trace exception generator
                EnableBreakPoints() ;
                SR = SR & (unsigned short int)(0x7FFF) ;    // clear T bit in status register
                printf("\r\nSingle Step : [OFF]") ;
                printf("\r\nBreak Points :[Enabled]") ;
                printf("\r\nPress <ESC> to Resume User Program.....") ;
            }
        }

        else if(c == (char)(0x1b))  {   // if user choses to end trace and run program
            Trace = 0;
            TraceException = 0;
            x = *(unsigned int *)(0x00000074) ;   // read IRQ 5 vector to reset trace vector generator
            EnableBreakPoints() ;
            SR = SR & (unsigned short int)(0x7FFF) ;    // clear T bit in status register

            printf("\r\nSingle Step  :[OFF]") ;
            printf("\r\nBreak Points :[Enabled]");
            printf("\r\nProgram Running.....") ;
            printf("\r\nPress <RESET> button <Key0> on DE1 to stop") ;
            return ;
        }

        else if( c == (char)('W'))              // Watchpoint command
            Watchpoint() ;

        // else if( c == (char)('I'))             // I2C EEPROM interfacing
        //      I2C_EEPROM() ;

        else if( c == (char)('I'))  {          // Test command
            
            c1 = toupper(_getch()) ;
            if(c1 == (char)('R'))                    // memory test
                I2C_EEPROM_RandomRead() ;
            else if( c1 == (char)('W'))              // Switch Test command
                I2C_EEPROM_RandomWrite() ;
            else if( c1 == (char)('1'))
                I2C_EEPROM_BLOCKWRITE() ;
            else if( c1 == (char)('2'))
                I2C_EEPROM_BLOCKREAD() ;
            else if( c1 == (char)('G'))
                PCF8591_Gen();
            else if( c1 == (char)('C'))
                PCF8591_Read();
            else
                UnknownCommand() ;
        }
        

        else
            UnknownCommand() ;
    }
}

void PrintErrorMessageandAbort(char *string) {
    printf("\r\n\r\nProgram ABORT !!!!!!\r\n") ;
    printf("%s\r\n", string) ;
    menu() ;
}

void IRQMessage(int level) {
     printf("\r\n\r\nProgram ABORT !!!!!");
     printf("\r\nUnhandled Interrupt: IRQ%d !!!!!", level) ;
     menu() ;
}

void UnhandledIRQ1(void) {
     IRQMessage(1);
}

void UnhandledIRQ2(void) {
    IRQMessage(2);
}

void UnhandledIRQ3(void){
    IRQMessage(3);
}

void UnhandledIRQ4(void) {
     IRQMessage(4);
}

void UnhandledIRQ5(void) {
    IRQMessage(5);
}

void UnhandledIRQ6(void) {
    PrintErrorMessageandAbort("ADDRESS ERROR: 16 or 32 Bit Transfer to/from an ODD Address....") ;
    menu() ;
}

void UnhandledIRQ7(void) {
    IRQMessage(7);
}

void UnhandledTrap(void) {
    PrintErrorMessageandAbort("Unhandled Trap !!!!!") ;
}

void BusError() {
   PrintErrorMessageandAbort("BUS Error!") ;
}

void AddressError() {
   PrintErrorMessageandAbort("ADDRESS Error!") ;
}

void IllegalInstruction() {
    PrintErrorMessageandAbort("ILLEGAL INSTRUCTION") ;
}

void Dividebyzero() {
    PrintErrorMessageandAbort("DIVIDE BY ZERO") ;
}

void Check() {
   PrintErrorMessageandAbort("'CHK' INSTRUCTION") ;
}

void Trapv() {
   PrintErrorMessageandAbort("TRAPV INSTRUCTION") ;
}

void PrivError() {
    PrintErrorMessageandAbort("PRIVILEGE VIOLATION") ;
}

void UnitIRQ() {
    PrintErrorMessageandAbort("UNINITIALISED IRQ") ;
}

void Spurious() {
    PrintErrorMessageandAbort("SPURIOUS IRQ") ;
}

void EnterString(void)
{
    unsigned char *Start;
    unsigned char c;

    printf("\r\nStart Address in Memory: ") ;
    Start = Get8HexDigits(0) ;

    printf("\r\nEnter String (ESC to end) :") ;
    while((c = getchar()) != 0x1b)
        *Start++ = c ;

    *Start = 0x00;  // terminate with a null
}


// automatic test functions
char datatypeselection (){
    int UserSelection = 0;
    char UserSelect = ' ';

    printf("\nSelect which data type sequence to test on memory:\n[A] Bytes (8 bits)\n[B] Words (16 bits)\n[C] Long words(32 bits)\n");
    FlushKeyboard() ;               // dump unread characters from keyboard
    printf("\r\n#") ;
    UserSelect = toupper(getchar());
    // scanf("%d", &UserSelection);

    if (UserSelect != (char)('A') && UserSelect!= (char)('B') && UserSelect!= (char)('C')){
        printf("Invalid selection, defaulting to testing with Bytes\n");
        UserSelect = (char)('A');
    }
    return UserSelect;
}

char testdataselection (){
    int UserSelection = 0;
    char UserSelect = ' ';

    printf("\nSelect which test data sequence to test on memory:\n[A]1234\n[B]abcd\n[C]5555\n[D]ffff\n");
    FlushKeyboard() ;               // dump unread characters from keyboard
    printf("\r\n#") ;
    UserSelect = toupper(getchar());

    if (UserSelect != (char)('A') && UserSelect != (char)('B') && UserSelect != (char)('C') && UserSelect != (char)('D')){
        printf("Invalid selection, defaulting to testing with option [A]\n");
        UserSelect = (char)('A');
    }
    return UserSelect;
}

void populatememory(int pattern, char datalength){
    // populate something here for writing to memory
    unsigned int* int_mem_pointer;
    unsigned char* char_mem_pointer;
    unsigned short int* shortint_mem_pointer;

    if (datalength == (char)('A')){ // bytes
        printf("Populating with bytes\n");
        for(char_mem_pointer = addressTestLower; char_mem_pointer<addressTestUpper; char_mem_pointer++){
            *char_mem_pointer = pattern;
        }
        printf("last address written to: %x\n", char_mem_pointer-1);
    }
    else if (datalength == (char)('B')){ // words
        printf("Populating with words\n");
        for(shortint_mem_pointer = addressTestLower; shortint_mem_pointer<addressTestUpper; shortint_mem_pointer++){// the ++ should already increment it nicely
            *shortint_mem_pointer = pattern;
        }
        printf("last address written to: %x\n", shortint_mem_pointer-2);
    }
    else { // long words
        printf("Populating with long words\n");
        for(int_mem_pointer = addressTestLower; int_mem_pointer<addressTestUpper; int_mem_pointer++){// the ++ should already increment it nicely
            *int_mem_pointer = pattern;
        }
        printf("last address written to: %x\n", int_mem_pointer-4);
    }
    printf("Population process complete\n");
}

// function which checks the contents of memory populated by the preceding function
void checkmemory(int pattern, char datalength){
    unsigned int* int_mem_pointer;
    unsigned char* char_mem_pointer;
    unsigned short int* shortint_mem_pointer;

    unsigned char bytebuffer;
    unsigned short int wordbuffer;
    unsigned int longwordbuffer;

    int internal_count = 0;


    if (datalength == (char)('A')) {
        // we know that we should be reading byte patterns
        bytebuffer = pattern;// assuming that the overflow bits are lost
        for(char_mem_pointer = addressTestLower; char_mem_pointer<addressTestUpper; char_mem_pointer++){// this pointer should increment nicely
            // if(char_mem_pointer % 1000== 0)
            // {
            //     printf("Addres: %x\n", char_mem_pointer);
            //     printf("Value at memory address: %x\n", *char_mem_pointer);
            // }
            if (internal_count == 1000){
                printf("Checking Address: %x\nValue retrieved: %x\n", char_mem_pointer, *char_mem_pointer);
                internal_count = 0;
            }
            if(*char_mem_pointer != bytebuffer){
                printf("Mismatch at address: %x\n Expected value: %x\nRead value:     %x\n\n", char_mem_pointer, bytebuffer, *char_mem_pointer);
                break;
            }
            internal_count++;
        }
    }
    else if (datalength == (char)('B')){
        // we know that we should be reading word patterns
        wordbuffer = pattern;// assuming that the overflow bits are lost
        for(shortint_mem_pointer = addressTestLower; shortint_mem_pointer<addressTestUpper; shortint_mem_pointer++){// this pointer should increment nicely
            if(*shortint_mem_pointer != wordbuffer){
                printf("Mismatch at address: %x\n Expected value: %x\nRead value:     %x\n\n", shortint_mem_pointer, wordbuffer, *shortint_mem_pointer);
                break;
            }
            if (internal_count == 1000){
                printf("Checking Address: %x\nValue retrieved: %x\n", shortint_mem_pointer, *shortint_mem_pointer);
                internal_count = 0;
            }
            internal_count++;
        }

    }
    else{ // datalength should be equal to 3
        // we know that we should be reading long word patterns
        longwordbuffer = pattern;// assuming that the overflow bits are lost
        for(int_mem_pointer = addressTestLower; int_mem_pointer<addressTestUpper; int_mem_pointer++){// this pointer should increment nicely
            if(*int_mem_pointer != longwordbuffer){
                printf("Mismatch at address: %x\n Expected value: %x\nRead value:     %x\n\n", int_mem_pointer, longwordbuffer, *int_mem_pointer);
                break;
            }
            if (internal_count == 1000){
                printf("Checking Address: %x\nValue retrieved: %x\n", int_mem_pointer, *int_mem_pointer);
                internal_count = 0;
            }
            internal_count++;
        }
    }
    printf("Check complete...\n");
}

void automatictests(){
    char userdatatype;
    char usertestdata;

    int selectedPattern;

    // get the selection of byte, word or long word test to be run
    userdatatype = datatypeselection(); // possible values 1-3
    usertestdata = testdataselection(); // possible values 1-4

    if (usertestdata == (char)('A')){
        selectedPattern = 0x12341234;
    }
    else if (usertestdata == (char)('B')){
        selectedPattern = 0xabcdabcd;
    }
    else if (usertestdata == (char)('C')){
        selectedPattern = 0x55555555;
    }
    else if (usertestdata == (char)('D')){
        selectedPattern = 0xffffffff;
    }

    // pass these to a write/read function

    populatememory(selectedPattern, userdatatype);
    checkmemory(selectedPattern, userdatatype);
}
// the type of pointer matters when trying to read or write to memory
// char pointers for bytes
// short int pointers for words
// int pointers for long words

void manualtests(){
    int pattern;
    char patternselection;
    int startAddress;
    int endAddress;
    char datalength;

    datalength = datatypeselection();
    patternselection = testdataselection();
    startAddress = useraddressStart();
    endAddress = useraddressEnd();

    if (patternselection == (char)('A')){
        pattern = 0x12341234;
        // printf("Pattern is %x\n", pattern);
    }
    else if (patternselection == (char)('B')){
        pattern = 0xabcdabcd;
        // printf("Pattern is %x\n", pattern);
    }
    else if (patternselection == (char)('C')){
        pattern = 0x55555555;
        // printf("Pattern is %x\n", pattern);
    }
    else if (patternselection == (char)('D')){
        pattern = 0xffffffff;
        // printf("Pattern is %x\n", pattern);
    }

    memorywrite(pattern, startAddress, endAddress, datalength);
    memoryread(startAddress, endAddress, datalength);
}

void memorywrite(int pattern, int startAddress, int endAddress, char datalength){
    // printf("Pattern is %x\n", pattern);
    unsigned int* int_mem_pointer;
    unsigned char* char_mem_pointer;
    unsigned short int* shortint_mem_pointer;

    if(startAddress >= addressTestUpper || startAddress < addressTestLower || endAddress >= addressTestUpper || endAddress < addressTestLower || startAddress > endAddress){
        printf("Invalid address for write\nWrite Operation Fail\n");
        return;
    }
    else{
        if(datalength == (char)('A')){
            unsigned char* address;
                    for(address = startAddress; address <= endAddress; address++)
                    {
                        char_mem_pointer = address;
                        *char_mem_pointer = pattern;
                    }
                    printf("\nByte write operation complete\n");
                }
        else if(datalength == (char)('B') || datalength == (char)('C')){ // need to check if the address is even or not
            if (startAddress%2 != 0 || endAddress%2 != 0){ // if the address is not even, fail to write
                printf("\nAddress for word or longword is not even\nWrite Operation Fail\n");
                return;
            }
            else if(datalength == (char)('C') && ((endAddress-startAddress)<4))
            {
                printf("\nAddress for longword is not long enough\nWrite Operation Fail\n");
                return;
            }

            else{
                // actually perform the write operation
                if (datalength == (char)('B')){
                    unsigned short int* address;
                    for(address = startAddress; address <= endAddress; address++)
                    {
                        shortint_mem_pointer = address;
                        *shortint_mem_pointer = pattern;
                    }
                    printf("\nWord write operation complete\n");
                }
                else{
                    unsigned int* address;
                    for(address = startAddress; address <= endAddress; address++)
                    {
                        int_mem_pointer = address;
                        *int_mem_pointer = pattern;
                    }
                    printf("\nLong word write operation complete\n");
                }
            }
        }
    }
    return;
}

void memoryread(int startAddress, int endAddress, char datalength){
    unsigned int* int_mem_pointer;
    unsigned char* char_mem_pointer;
    unsigned short int* shortint_mem_pointer;

    unsigned int universalbuffer = 0;

    if(datalength == (char)('A')){
        unsigned char* address;
        printf("data type selected: Byte\n",datalength);
        for(address = startAddress; address <= endAddress; address++)
        {
            char_mem_pointer = address;
            universalbuffer = *char_mem_pointer;
            printf("Read address: %x\nByte value: %x\n", address, universalbuffer);
        }
    }
    else if (datalength == (char)('B')){
        unsigned short int* address;
        printf("data type selected: Word\n");
        for(address = startAddress; address <= endAddress; address++)
        {
            shortint_mem_pointer = address;
            universalbuffer = *shortint_mem_pointer;
            printf("Read address: %x\nWord value: %x\n", address, universalbuffer);
        }
    }
    else{
        unsigned int* address;
        printf("data type selected: Long Word\n");
        for(address = startAddress; address <= endAddress; address++)
        {
            int_mem_pointer = address;
            universalbuffer = *int_mem_pointer;
            printf("Read address: %x\nLong word value: %x\n", address, universalbuffer);
        }
    }
    return;
}

int useraddressStart(){
    unsigned int address;
    printf("\nEnter start address to be tested (hex):\n");
    address = Get8HexDigits(0);
    return address;
}

int useraddressEnd(){
    unsigned int address;
    printf("\nEnter end address to be tested (hex):\n");
    address = Get8HexDigits(0);
    return address;
}


void MemoryTest(void)
{
    // unsigned int *RamPtr, counter1=1 ;
    // register unsigned int i ;
    // unsigned int Start, End ;
    // char c ;

    // printf("\r\nStart Address: ") ;
    // Start = Get8HexDigits(0) ;
    // printf("\r\nEnd Address: ") ;
    // End = Get8HexDigits(0) ;
        // variable declarations
    int usertestmode = 0;

    // memory pointers
    char* byte_mem_pointer;
    char holderChar = ' ';
	// TODO

	// add your code to test memory here using 32 bit reads and writes of data between the start and end of memory
     //memory clear
    FlushKeyboard();
    // for (byte_mem_pointer = addressTestLower; byte_mem_pointer < addressTestUpper; byte_mem_pointer++) // populating every two addresses for test purposes
    //     {
    //         *byte_mem_pointer = 0; // set every byte in memory to zero

    //     }
    printf("Cleared memory\n");
    FlushKeyboard();
    printf("Choose test mode:\n[A]Automatic Tests (write to all memory addresses and read)\n[B]Manual Test (select a memory address range)\n");
    FlushKeyboard() ;               // dump unread characters from keyboard
    printf("\r\n#") ;
    holderChar = toupper(getchar());

    if(holderChar == (char)('A')){
        automatictests();
    }
    else if(holderChar == (char)('B')){
        manualtests();
    }
}

void main(void)
{

    char c ;
    int i, j ;

    char *BugMessage = "DE1-68k Bug V1.77 - Lab 5 Completed";
    char *CopyrightMessage = "Warren Chan 96150818 Jordan Lee 98807845";

    KillAllBreakPoints() ;

    i = x = y = z = PortA_Count = 0;
    Trace = GoFlag = 0;                       // used in tracing/single stepping
    Echo = 1 ;

    d0=d1=d2=d3=d4=d5=d6=d7=0 ;
    a0=a1=a2=a3=a4=a5=a6=0 ;


    PC = ProgramStart, SSP=TopOfStack, USP = TopOfStack;
    SR = 0x2000;                            // clear interrupts enable tracing  uses IRQ6

// Initialise Breakpoint variables

    for(i = 0; i < 8; i++)  {
        BreakPointAddress[i] = 0;               //array of 8 breakpoint addresses
        WatchPointAddress[i] = 0 ;
        BreakPointInstruction[i] = 0;           // to hold the instruction at the break point
        BreakPointSetOrCleared[i] = 0;          // indicates if break point set
        WatchPointSetOrCleared[i] = 0;
    }

    Init_RS232() ;     // initialise the RS232 port
    Init_LCD() ;

    for( i = 32; i < 48; i++)
       InstallExceptionHandler(UnhandledTrap, i) ;		        // install Trap exception handler on vector 32-47

    InstallExceptionHandler(menu, 47) ;		                   // TRAP #15 call debug and end program
    InstallExceptionHandler(UnhandledIRQ1, 25) ;		      // install handler for interrupts
    InstallExceptionHandler(UnhandledIRQ2, 26) ;		      // install handler for interrupts
    InstallExceptionHandler(UnhandledIRQ3, 27) ;		      // install handler for interrupts
    InstallExceptionHandler(UnhandledIRQ4, 28) ;		      // install handler for interrupts
    InstallExceptionHandler(UnhandledIRQ5, 29) ;		      // install handler for interrupts
    InstallExceptionHandler(UnhandledIRQ6, 30) ;		      // install handler for interrupts
    InstallExceptionHandler(UnhandledIRQ7, 31) ;		      // install handler for interrupts


    InstallExceptionHandler(HandleBreakPoint, 46) ;		           // install Trap 14 Break Point exception handler on vector 46
    InstallExceptionHandler(DumpRegistersandPause, 29) ;		   // install TRACE handler for IRQ5 on vector 29

    InstallExceptionHandler(BusError,2) ;                          // install Bus error handler
    InstallExceptionHandler(AddressError,3) ;                      // install address error handler (doesn't work on soft core 68k implementation)
    InstallExceptionHandler(IllegalInstruction,4) ;                // install illegal instruction exception handler
    InstallExceptionHandler(Dividebyzero,5) ;                      // install /0 exception handler
    InstallExceptionHandler(Check,6) ;                             // install check instruction exception handler
    InstallExceptionHandler(Trapv,7) ;                             // install trapv instruction exception handler
    InstallExceptionHandler(PrivError,8) ;                         // install Priv Violation exception handler
    InstallExceptionHandler(UnitIRQ,15) ;                          // install uninitialised IRQ exception handler
    InstallExceptionHandler(Check,24) ;                            // install spurious IRQ exception handler


    FlushKeyboard() ;                        // dump unread characters from keyboard
    TraceException = 0 ;                     // clear trace exception port to remove any software generated single step/trace

    SPI_Init();
    // test for auto flash boot and run from Flash by reading switch 9 on DE1-soc board. If set, copy program from flash into Dram and run


    // writes to the control register and sets the prescaling for the SCL frequency
    I2C_Init(); // need to check that this is the right location for iniatilzing the I2C core
    // CanBusTest();
    Init_CanBus_Controller0();
    Init_CanBus_Controller1();

    while(((char)(PortB & 0x02)) == (char)(0x02))    {
        LoadFromFlashChip();
        printf("\r\nRunning.....") ;
        Oline1("Running.....") ;
        GoFlag = 1;
        go() ;
    }

    // otherwise start the debug monitor

    Oline0(BugMessage) ;
    Oline1("By: PJ Davies") ;

    printf("\r\n%s", BugMessage) ;
    printf("\r\n%s", CopyrightMessage) ;

    menu();
}

