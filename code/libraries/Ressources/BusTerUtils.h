// File     : BusTerUtils.h
// Date     : 14.04.2016
// 
// Goal     : This library is made to be used with a Arduino and adapted for 
//            the laboratory BusTer.
//
//            This library contains :
//             - All features to manage digital and analog IO.
//             - Can functions.
// 
// Note : This library has been built from the following libraries: mcp_can.h, 
//        mcp_can_dfs.h, mcp4728.h
// 

#ifndef _BUSTER_UTILS_H_
#define _BUSTER_UTILS_H_

#include "Arduino.h"

#ifndef INT32U
#define INT32U unsigned long
#endif

#ifndef INT8U
#define INT8U byte
#endif

// Default Modbus port
#define BUSTER_MODBUS_PORT 502

// Define the pins used on Arduino

// Digital inputs
#define BUSTER_PIN_IN_0    40
#define BUSTER_PIN_IN_1    41
#define BUSTER_PIN_IN_2    38
#define BUSTER_PIN_IN_3    39
#define BUSTER_PIN_IN_4    36
#define BUSTER_PIN_IN_5    37
#define BUSTER_PIN_IN_6    34
#define BUSTER_PIN_IN_7    35

// Digital outputs
#define BUSTER_PIN_OUT_0 48
#define BUSTER_PIN_OUT_1 49
#define BUSTER_PIN_OUT_2 46
#define BUSTER_PIN_OUT_3 47
#define BUSTER_PIN_OUT_4 44
#define BUSTER_PIN_OUT_5 45
#define BUSTER_PIN_OUT_6 42
#define BUSTER_PIN_OUT_7 43
       
// Dip switch
#define BUSTER_PIN_DIPSWITCH_1 32
#define BUSTER_PIN_DIPSWITCH_2 33
#define BUSTER_PIN_DIPSWITCH_3 30
#define BUSTER_PIN_DIPSWITCH_4 31
#define BUSTER_PIN_DIPSWITCH_5 28
#define BUSTER_PIN_DIPSWITCH_6 29
#define BUSTER_PIN_DIPSWITCH_7 26
#define BUSTER_PIN_DIPSWITCH_8 27

// Status led
#define BUSTER_PIN_STATUS_LED_GREEN 24
#define BUSTER_PIN_STATUS_LED_RED 25

enum BUSTER_AnalogInput  
{ 
    BUSTER_ANALOGINPUT_0, BUSTER_ANALOGINPUT_1, 
    BUSTER_ANALOGINPUT_2, BUSTER_ANALOGINPUT_3
};

// ****************************************************************************
// Function name: BusTer_InitInputOutput
// Descriptions:  Init the input and output of the arduino
// ****************************************************************************
void BusTer_InitInputOutput();

// ****************************************************************************
// Function name: BusTer_AnalogRead
// Descriptions:  Read an analog input corresponding to the input parameter.
// return value:  returns the input voltage in mV, range 0..5000. 
// ****************************************************************************
int BusTer_AnalogRead(BUSTER_AnalogInput input);

// ****************************************************************************
// Function name: BusTer_AnalogWrite
// Descriptions:  Write the 4 analog outputs 
//                Values are in [mV] in the range of 0..5000 mV
// ****************************************************************************
void BusTer_AnalogWrite(int output0, int output1, int output2, int output3);

// *************************** API CAN for Arduino ****************************

// Can bit rates
enum BUSTER_CanSpeed 
{ 
    BUSTER_CAN_5KBPS = 1, BUSTER_CAN_10KBPS, BUSTER_CAN_20KBPS, 
    BUSTER_CAN_31K25BPS, BUSTER_CAN_33KBPS, BUSTER_CAN_40KBPS, 
    BUSTER_CAN_50KBPS, BUSTER_CAN_80KBPS, BUSTER_CAN_83K3BPS, 
    BUSTER_CAN_95KBPS, BUSTER_CAN_100KBPS, BUSTER_CAN_125KBPS, 
    BUSTER_CAN_200KBPS, BUSTER_CAN_250KBPS, BUSTER_CAN_500KBPS, 
    BUSTER_CAN_1MBPS 
};
                       
// Can formats
enum BUSTER_CanType
{ 
    BUSTER_Can2_0A = 0, BUSTER_Can2_0B 
}; 

// Receive can buffer registers
enum BUSTER_CanRxb 
{ 
    BUSTER_Rxb0, BUSTER_Rxb1 
}; 

// Receive mask registers
enum BUSTER_CanRxm 
{ 
    BUSTER_Rxm0, BUSTER_Rxm1 
}; 

// Receive filter registers
enum BUSTER_CanRxf 
{ 
    BUSTER_Rxf0, BUSTER_Rxf1, BUSTER_Rxf2, BUSTER_Rxf3, BUSTER_Rxf4, 
    BUSTER_Rxf5 
}; 

// Transmit buffers registers
enum BUSTER_CanTxb 
{ 
    BUSTER_Txb0 = 0x30, BUSTER_Txb1 = 0x40, BUSTER_Txb2 = 0x50 
}; 

// ****************************************************************************
// Function name:           BusTer_CanInit
// Descriptions:            Init the MCP2515 can controler with the canSpeed 
//                          Set the controller in configuration mode  
// ****************************************************************************
void  BusTer_CanInit(BUSTER_CanSpeed canSpeed);                                    

// ****************************************************************************
// Function name:       BusTer_CanSetMask
// Parameter rxMask:    the mask register to set
// Parameter type:      Specification CAN_2.0A or CAN_2.0B
// Parameter ulData:    mask to init
// Descriptions:        init canid masks, returns true if success
// ****************************************************************************
bool BusTer_CanSetMask(BUSTER_CanRxm rxMask, BUSTER_CanType type, 
    INT32U ulData);

// ****************************************************************************
// Function name:      BusTer_CanSetFilter
// Parameter rxFilter: the filter register to set
// Parameter type:     Message format CAN_2.0A or CAN_2.0B
// Parameter ulData:   filter to init
// Descriptions:       init canid filters, returns true if success
// ****************************************************************************
bool BusTer_CanSetFilter(BUSTER_CanRxf rxFilter, BUSTER_CanType type, 
    INT32U ulData);

// ****************************************************************************
// Function name:      BusTer_CanActivateNormalMode
// Descriptions:       Set the MCP2515 in normal operation mode
// ****************************************************************************
bool BusTer_CanActivateNormalMode();

// ****************************************************************************
// Function name:    BusTer_CanSend
// parameter Txbuf : Number of the transmit buffer (ENUM)
// parameter type :  Specification CAN_2.0A or CAN_2.0B (ENUM)
// parameter arb :   arbitration field
// parameter len :   length of the data buffer
// parameter *buf :  data
// Descriptions:     write message to send buffer.
//                   returns true if success and false if timeout.
// ****************************************************************************
bool  BusTer_CanSend(BUSTER_CanTxb Txbuf, BUSTER_CanType type, INT32U arb, 
    INT8U len, INT8U *buf); 

// ****************************************************************************
// Function name: BusTer_CanSendReady
// Descriptions:  returns true if the transmit buffer (txbuf_n) is 
//                ready to send a new message
// ****************************************************************************
bool  BusTer_CanSendReady(BUSTER_CanTxb txbuf_n);

// ****************************************************************************
// Function name:       BusTer_CanMessageReceived
// parameter RxbBuffer: buffer to check if a message is received
// Descriptions:        returns true if a message has been received
//*****************************************************************************
bool  BusTer_CanMessageReceived(BUSTER_CanRxb RxbBuffer);

// ****************************************************************************
// Function name:        BusTer_CanRead
// parameter RxbBuffer : Read in the following buffer
// parameter arb :       return the arbitration field received
// parameter len :       return the length of the data received
// parameter buf[] :     adress of the buffer to put the data
// Descriptions:         reads message buf and can bus source arb, 
//                       Returns true if success
// ****************************************************************************
bool BusTer_CanRead(BUSTER_CanRxb RxbBuffer, INT32U *arb, INT8U *len, 
    INT8U *buf);

// ****************************************************************************
// Function name:           BusTer_CanHasError
// Descriptions:            if can communication in error, returns true
// ****************************************************************************
bool  BusTer_CanHasError();  

// ****************************************************************************
// Function name:           BusTer_CanClearError
// Descriptions:            set all message to zero and clear error
// ****************************************************************************
void BusTer_CanClearError();

#endif
