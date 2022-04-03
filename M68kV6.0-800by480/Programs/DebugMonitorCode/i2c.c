
#include "i2c.h"
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


void PotRead(void){
    unsigned char read_byte;
    int i =0;
    I2C_wait_transmit_finish();

    printf("\r\nReading ADC channels\r\n");

    I2C_CORE_TRANSMIT = ADC_SLAVE_ADDRESS_MWRITE; //write initial address
    I2C_CORE_COMMAND = I2C_SlaveWriteStart;

    I2C_wait_transmit_finish();
    I2C_check_slave_acknowledge();

    I2C_CORE_TRANSMIT = ADC_ANALOG_SINGULARREAD_CTRL_BYTE_POTE;
    I2C_CORE_COMMAND = I2C_SlaveWrite;

    I2C_wait_transmit_finish();
    I2C_check_slave_acknowledge();

    I2C_CORE_TRANSMIT = ADC_SLAVE_ADDRESS_MREAD; //write initial address
    I2C_CORE_COMMAND = I2C_SlaveWriteStart;

    I2C_wait_transmit_finish();
    I2C_check_slave_acknowledge();


    I2C_CORE_COMMAND = I2C_SlaveRead;
    I2C_wait_transmit_finish();

    while(I2C_CORE_STATUS & 0x01 == 0x00);// will stay here unless the core status bit 0 is set to 1

    read_byte = I2C_CORE_RECEIVE;
    printf("\r\n Pot read is: %d\r\n", read_byte);
    I2C_CORE_COMMAND = I2C_SlaveStop;
    I2C_wait_transmit_finish();

}

void ThermRead(void){
    unsigned char read_byte;
    int i =0;
    I2C_wait_transmit_finish();

    printf("\r\nReading ADC channels\r\n");

    I2C_CORE_TRANSMIT = ADC_SLAVE_ADDRESS_MWRITE; //write initial address
    I2C_CORE_COMMAND = I2C_SlaveWriteStart;

    I2C_wait_transmit_finish();
    I2C_check_slave_acknowledge();

    I2C_CORE_TRANSMIT = ADC_ANALOG_SINGULARREAD_CTRL_BYTE_THER;
    I2C_CORE_COMMAND = I2C_SlaveWrite;

    I2C_wait_transmit_finish();
    I2C_check_slave_acknowledge();

    I2C_CORE_TRANSMIT = ADC_SLAVE_ADDRESS_MREAD; //write initial address
    I2C_CORE_COMMAND = I2C_SlaveWriteStart;

    I2C_wait_transmit_finish();
    I2C_check_slave_acknowledge();


    I2C_CORE_COMMAND = I2C_SlaveRead;
    I2C_wait_transmit_finish();

    while(I2C_CORE_STATUS & 0x01 == 0x00);// will stay here unless the core status bit 0 is set to 1

    read_byte = I2C_CORE_RECEIVE;
    printf("\r\n Thermistor read is: %d\r\n", read_byte);
    I2C_CORE_COMMAND = I2C_SlaveStop;
    I2C_wait_transmit_finish();

}

void PhotoRead(void){
    unsigned char read_byte;
    int i =0;
    I2C_wait_transmit_finish();

    printf("\r\nReading ADC channels\r\n");

    I2C_CORE_TRANSMIT = ADC_SLAVE_ADDRESS_MWRITE; //write initial address
    I2C_CORE_COMMAND = I2C_SlaveWriteStart;

    I2C_wait_transmit_finish();
    I2C_check_slave_acknowledge();

    I2C_CORE_TRANSMIT = ADC_ANALOG_SINGULARREAD_CTRL_BYTE_LITE;
    I2C_CORE_COMMAND = I2C_SlaveWrite;

    I2C_wait_transmit_finish();
    I2C_check_slave_acknowledge();

    I2C_CORE_TRANSMIT = ADC_SLAVE_ADDRESS_MREAD; //write initial address
    I2C_CORE_COMMAND = I2C_SlaveWriteStart;

    I2C_wait_transmit_finish();
    I2C_check_slave_acknowledge();


    I2C_CORE_COMMAND = I2C_SlaveRead;
    I2C_wait_transmit_finish();

    while(I2C_CORE_STATUS & 0x01 == 0x00);// will stay here unless the core status bit 0 is set to 1

    read_byte = I2C_CORE_RECEIVE;
    printf("\r\n Photoresistor read is: %d\r\n", read_byte);
    I2C_CORE_COMMAND = I2C_SlaveStop;
    I2C_wait_transmit_finish();

}

unsigned char PCF8591_Read(unsigned char Timer5Count){
    unsigned char read_byte;
    int i =0;
    unsigned char dataArray[3];
    I2C_wait_transmit_finish();



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
            dataArray[0] = read_byte;
        }
        else if(i == 2){
            dataArray[1] = read_byte;
        }
        else if (i==3){
            dataArray[2] = read_byte;
        }
    }
    if(Timer5Count == 1){
        // printf("\r\nThermistor read is: %d\r\n", dataArray[0]);
        return dataArray[0];
    }
    else if(Timer5Count == 2){
        // printf("\r\nPotentiometer read is: %d\r\n", dataArray[1]);
        return dataArray[1];
    }
    else if (Timer5Count==3){
        // printf("\r\nPhotoresistor read is: %d\r\n", dataArray[2]);
        return dataArray[2];
    }
    I2C_CORE_COMMAND = I2C_SlaveStop;
    I2C_wait_transmit_finish();

}