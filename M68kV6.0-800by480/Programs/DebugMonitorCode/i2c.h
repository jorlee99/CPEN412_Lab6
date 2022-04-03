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
#define ADC_ANALOG_READ_CTRL_BYTE 0x44
#define ADC_ANALOG_SINGULARREAD_CTRL_BYTE_POTE 0x41 // no auto increment flag
#define ADC_ANALOG_SINGULARREAD_CTRL_BYTE_LITE 0x42 // no auto increment flag
#define ADC_ANALOG_SINGULARREAD_CTRL_BYTE_THER 0x43 // no auto increment flag

void I2C_check_slave_acknowledge(void);
void I2C_Init(void);
void I2C_wait_transmit_finish(void);
void PCF8591_Read(unsigned char);
void PotRead(void);
void PhotoRead(void);
void ThermRead(void);

