// #include "CanBus1.h"
// // initialisation for Can controller 0
// void Init_CanBus_Controller0(void)
// {
//     // TODO - put your Canbus initialisation code for CanController 0 here
//     // See section 4.2.1 in the application note for details (PELICAN MODE)
// 	// define interrupt priority level and control
// 	while((	Can0_ModeControlReg & RM_RR_Bit) == ClrByte){
// 		Can0_ModeControlReg = (Can0_ModeControlReg | RM_RR_Bit);
// 	}

// 	// set up clock divider
// 	Can0_ClockDivideReg = (CANMode_Bit | CBP_Bit | DivBy2);
// 	// disable can interrupt
// 	Can0_InterruptEnReg = ClrIntEnSJA;

//     Can0_AcceptCode0Reg = ClrByte;
//     Can0_AcceptCode1Reg = ClrByte;
//     Can0_AcceptCode2Reg = ClrByte;
//     Can0_AcceptCode3Reg = ClrByte;
//     Can0_AcceptMask0Reg = DontCare;
//     Can0_AcceptMask1Reg = DontCare;
//     Can0_AcceptMask2Reg = DontCare;
//     Can0_AcceptMask3Reg = DontCare;

//     // configure bus timing
//     Can0_BusTiming0Reg = 0x04;
//     Can0_BusTiming1Reg = 0x7f;

//     Can0_OutControlReg = (Tx1Float | Tx0PshPull | NormalMode);

//     do{
//         Can0_ModeControlReg = ClrByte;
//     }while((Can0_ModeControlReg & RM_RR_Bit) != ClrByte);
//     printf("Finished Init of CAN 0\r\n");
// }

// // initialisation for Can controller 1
// void Init_CanBus_Controller1(void)
// {
//     // TODO - put your Canbus initialisation code for CanController 1 here
//     // See section 4.2.1 in the application note for details (PELICAN MODE)
//     // TODO - put your Canbus initialisation code for CanController 0 here
//     // See section 4.2.1 in the application note for details (PELICAN MODE)
// 	// define interrupt priority level and control
// 	while((	Can1_ModeControlReg & RM_RR_Bit) == ClrByte){
// 		Can1_ModeControlReg = (Can1_ModeControlReg | RM_RR_Bit);
// 	}

// 	// set up clock divider
// 	Can1_ClockDivideReg = CANMode_Bit | CBP_Bit | DivBy2;
// 	// disable can interrupt
// 	Can1_InterruptEnReg = ClrIntEnSJA;

//     Can1_AcceptCode0Reg = ClrByte;
//     Can1_AcceptCode1Reg = ClrByte;
//     Can1_AcceptCode2Reg = ClrByte;
//     Can1_AcceptCode3Reg = ClrByte;
//     Can1_AcceptMask0Reg = DontCare;
//     Can1_AcceptMask1Reg = DontCare;
//     Can1_AcceptMask2Reg = DontCare;
//     Can1_AcceptMask3Reg = DontCare;

//     // configure bus timing
//     Can1_BusTiming0Reg = 0x04;
//     Can1_BusTiming1Reg = 0x7f;

//     Can1_OutControlReg = Tx1Float | Tx0PshPull | NormalMode;

//     do{
//         Can1_ModeControlReg = ClrByte;
//     }while((Can1_ModeControlReg & RM_RR_Bit) != ClrByte);
//     printf("Finished Init of CAN 1\r\n");
// }

// // Transmit for sending a message via Can controller 0
// void CanBus0_Transmit(unsigned char byteToSend)
// {
//     // TODO - put your Canbus transmit code for CanController 0 here
//     // See section 4.2.2 in the application note for details (PELICAN MODE)

//     printf("\r\nByte to send : %x \n\r", byteToSend) ;

//     do{
//     }while((Can0_StatusReg & TBS_Bit) != TBS_Bit);

//     Can0_TxFrameInfo = 0x08; // # of bytes
//     Can0_TxBuffer1 = 0xA5;
//     Can0_TxBuffer2 = 0x20;

//     Can0_TxBuffer3 = 0x57; // Databits
//     Can0_TxBuffer4 = 0x41;
//     Can0_TxBuffer5 = 0x52;
//     Can0_TxBuffer6 = 0x52;
//     Can0_TxBuffer7 = 0x45;
//     Can0_TxBuffer8 = 0x4E;
//     Can0_TxBuffer9 = 0x20;
//     Can0_TxBuffer10 = byteToSend;

//     Can0_CommandReg = TR_Bit;
// }

// // Transmit for sending a message via Can controller 1
// void CanBus1_Transmit(unsigned char byteToSend)
// {
//     // TODO - put your Canbus transmit code for CanController 1 here
//     // See section 4.2.2 in the application note for details (PELICAN MODE)
//     printf("\r\nByte to send : %x \n\r", byteToSend) ;
//     do{
//     }while((Can1_StatusReg & TBS_Bit) != TBS_Bit);

    

//     Can1_TxFrameInfo = 0x08;// # of bytes

//     Can1_TxBuffer1 = 0xA5;
//     Can1_TxBuffer2 = 0x20;

//     Can1_TxBuffer3 = 0x4A; // start trans
//     Can1_TxBuffer4 = 0x4F;
//     Can1_TxBuffer5 = 0x52;
//     Can1_TxBuffer6 = 0x44;
//     Can1_TxBuffer7 = 0x41;
//     Can1_TxBuffer8 = 0x4E;
//     Can1_TxBuffer9 = 0x20;
//     Can1_TxBuffer10 = byteToSend;

//     Can1_CommandReg = TR_Bit;
// }

// // Receive for reading a received message via Can controller 0
// void CanBus0_Receive(void)
// {
//     // TODO - put your Canbus receive code for CanController 0 here
//     // See section 4.2.4 in the application note for details (PELICAN MODE)
//     unsigned char BitArray[7];

//     do{
//     }while( (Can0_StatusReg & RBS_Bit) != RBS_Bit);

//     BitArray[0] = Can0_RxBuffer1 & 0xff;
//     BitArray[1] = Can0_RxBuffer2 & 0xff;

//     //data bits
//     BitArray[2] = Can0_RxBuffer3;
//     BitArray[3] = Can0_RxBuffer4;
//     BitArray[4] = Can0_RxBuffer5;
//     BitArray[5] = Can0_RxBuffer6;
//     BitArray[6] = Can0_RxBuffer7;
//     BitArray[7] = Can0_RxBuffer8;
//     BitArray[8] = Can0_RxBuffer9;
//     BitArray[9] = Can0_RxBuffer10;
    

//     Can1_CommandReg = Can0_CommandReg & RRB_Bit;
//     printf("Can0RV : %c %c %c %c %c %c %c %c\n", BitArray[2], BitArray[3], BitArray[4], BitArray[5], BitArray[6],BitArray[7],BitArray[8],BitArray[9]);
// }

// // Receive for reading a received message via Can controller 1
// void CanBus1_Receive(void)
// {
//     // TODO - put your Canbus receive code for CanController 1 here
//     // See section 4.2.4 in the application note for details (PELICAN MODE)
//     unsigned char BitArray[7];

//     do{
//     }while( (Can1_StatusReg & RBS_Bit) != RBS_Bit);

//     BitArray[0] = Can1_RxBuffer1 & 0xff;
//     BitArray[1] = Can1_RxBuffer2 & 0xff;

//     //data bits
//     BitArray[2] = Can1_RxBuffer3;
//     BitArray[3] = Can1_RxBuffer4;
//     BitArray[4] = Can1_RxBuffer5;
//     BitArray[5] = Can1_RxBuffer6;
//     BitArray[6] = Can1_RxBuffer7;
//     BitArray[7] = Can1_RxBuffer8;
//     BitArray[8] = Can1_RxBuffer9;
//     BitArray[9] = Can1_RxBuffer10;

//     Can1_CommandReg = Can1_CommandReg & RRB_Bit;
//     printf("Can1RV : %c %c %c %c %c %c %c %c\n", BitArray[2], BitArray[3], BitArray[4], BitArray[5], BitArray[6],BitArray[7],BitArray[8],BitArray[9]);
// }


// // void CanBusTest(void)
// // {
// //     // initialise the two Can controllers
// //     // printf("Initializing canbus\r\n");
// //     int i =0;
// //     Init_CanBus_Controller0();
// //     Init_CanBus_Controller1();

// //     printf("\r\n\r\n---- CANBUS Test ----\r\n") ;

// //     // simple application to alternately transmit and receive messages from each of two nodes

// //     while(1)    {
// //         for(i = 0; i <500000; i++);                    // write a routine to delay say 1/2 second so we don't flood the network with messages to0 quickly

// //         CanBus0_Transmit() ;       // transmit a message via Controller 0
// //         CanBus1_Receive() ;        // receive a message via Controller 1 (and display it)

// //         printf("\r\n") ;

// //         for(i = 0; i <500000; i++);                    // write a routine to delay say 1/2 second so we don't flood the network with messages to0 quickly

// //         CanBus1_Transmit() ;        // transmit a message via Controller 1
// //         CanBus0_Receive() ;         // receive a message via Controller 0 (and display it)
// //         printf("\r\n") ;

// //     }
// // }


// void CanBusTest(void)
// {
//     // initialise the two Can controllers
//     int i = 0;
//     unsigned char testbyte;
//     // printf("Initializing canbus\r\n");

//     printf("\r\n\r\n---- CANBUS Test Warren ----\r\n") ;

//     // simple application to alternately transmit and receive messages from each of two nodes

//     // while(1)    {
//     //     for(i = 0; i <500000; i++);                    // write a routine to delay say 1/2 second so we don't flood the network with messages to0 quickly

//     //     CanBus0_Transmit(0x2a) ;       // transmit a message via Controller 0
//     //     CanBus1_Receive() ;        // receive a message via Controller 1 (and display it)

//     //     printf("\r\n") ;

//     //     for(i = 0; i <500000; i++);                    // write a routine to delay say 1/2 second so we don't flood the network with messages to0 quickly

//     //     CanBus1_Transmit(0x2b) ;        // transmit a message via Controller 1
//     //     CanBus0_Receive() ;         // receive a message via Controller 0 (and display it)
//     //     printf("\r\n") ;

//     // }
    

//     testbyte = 0x46; 
    
//     while(1)    {
//         for(i = 0; i <500000; i++);                    // write a routine to delay say 1/2 second so we don't flood the network with messages to0 quickly

//         printf("\r\nTest byte : %x \n\r", testbyte) ;

//         CanBus0_Transmit(testbyte) ;       // transmit a message via Controller 0
//         CanBus1_Receive() ;        // receive a message via Controller 1 (and display it)

//         printf("\r\n") ;

//         for(i = 0; i <500000; i++);                    // write a routine to delay say 1/2 second so we don't flood the network with messages to0 quickly

//         CanBus1_Transmit(testbyte) ;        // transmit a message via Controller 1
//         CanBus0_Receive() ;         // receive a message via Controller 0 (and display it)
//         printf("\r\n") ;


//     }

//     // modifications to the canbustest show that writing a byte using a variable works at least from the debug monitor
// }