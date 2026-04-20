/* File     : BusTerUtils.c
 * Autor    : Lionel von Niederhäusern
 * Date     : 24.03.2016
 *
 * Goal     : This library is made to be used with a Arduino and adapted for
 *            the laboratory BusTer.
 *            This library contains :
 *              - All features to manage digital or analog IO.
 *              - Can functions adapted for the labo.
 *
 * Note : This library has been adapted from the following libraries: mcp_can.h, mcp_can_dfs.h, mcp4728.h
 *
 * Modifications : 
*/
#include "BusTerUtils.h"
#include <SPI.h>
#include <Wire.h>

// Constants for the Analog output - MCP4728
#define defaultVDD 5001
#define BASE_ADDR 0x60
#define VREFWRITE 0B10000000
#define SINGLEWRITE 0B01011000

// Constants for the can communication - MCP2515
#define MAX_CHAR_IN_MESSAGE 8
#define CAN_MAX_CHAR_IN_MESSAGE (8)
#define CAN_OK                  (0)
#define CAN_FAILINIT            (1)
#define CAN_FAILTX              (2)
#define CAN_NOMSG               (4)
#define CAN_GETTXBFTIMEOUT      (6)
#define CAN_SENDMSGTIMEOUT      (7)
#define CAN_FAIL                (0xff)

#define MCP2515_OK         (0)
#define MCP2515_FAIL       (1)
#define TIMEOUTVALUE        50
#define MCP_SIDH        0
#define MCP_SIDL        1
#define MCP_EID8        2
#define MCP_EID0        3
#define MCP_STAT_RX0IF (1<<0)
#define MCP_STAT_RX1IF (1<<1)
#define MCP_RX0IF       0x01
#define MCP_RX1IF       0x02
#define MCP_CANINTF     0x2C
#define MCP_EFLG        0x2D
#define MODE_CONFIG     0x80
#define MODE_MASK       0xE0
#define MODE_NORMAL     0x00
#define MCP_CANCTRL     0x0F
#define MCP_CNF3        0x28
#define MCP_CNF2        0x29
#define MCP_CNF1        0x2A
#define MCP_RXM0SIDH    0x20
#define MCP_RXM1SIDH    0x24
#define MCP_RXF0SIDH    0x00
#define MCP_RXF1SIDH    0x04
#define MCP_RXF2SIDH    0x08
#define MCP_RXF3SIDH    0x10
#define MCP_RXF4SIDH    0x14
#define MCP_RXF5SIDH    0x18
#define MCP_RXB0SIDH    0x61
#define MCP_RXB1CTRL    0x70
#define MCP_RXB1SIDH    0x71
#define MCP_TXB0CTRL    0x30
#define MCP_TXB1CTRL    0x40
#define MCP_TXB2CTRL    0x50
#define MCP_RXB0CTRL    0x60
#define MCP_CANINTE     0x2B
#define MCP_TXB_TXREQ_M     0x08
#define MCP_DLC_MASK        0x0F                                        /* 4 LSBits                     */
#define MCP_RTR_MASK        0x40                                        /* (1<<6) Bit 6                 */
#define MCP_WRITE           0x02
#define MCP_READ            0x03
#define MCP_BITMOD          0x05
#define MCP_RESET           0xC0
#define MCP_READ_STATUS     0xA0
#define MCP_TXB_EXIDE_M     0x08                                        /* In TXBnSIDL                  */
#define MCP_RXB_RX_STDEXT   0x00
#define MCP_RXB_RX_MASK     0x60
#define MCP_STAT_RXIF_MASK  (0x03)
#define MCP_RXB_BUKT_MASK   (1<<2)


#define MCP_EFLG_ERRORMASK  (0x38)                                      /* 5 MS-Bits                    */
//#define MCP_EFLG_ERRORMASK  (0xF8)                                      /* 5 MS-Bits                    */
#define MCP_RXBUF_0 (MCP_RXB0SIDH)
#define MCP_RXBUF_1 (MCP_RXB1SIDH)
/*
 *  speed 16M
 */
#define MCP_16MHz_1000kBPS_CFG1 (0x00)
#define MCP_16MHz_1000kBPS_CFG2 (0xD0)
#define MCP_16MHz_1000kBPS_CFG3 (0x82)

#define MCP_16MHz_500kBPS_CFG1 (0x00)
#define MCP_16MHz_500kBPS_CFG2 (0xF0)
#define MCP_16MHz_500kBPS_CFG3 (0x86)

#define MCP_16MHz_250kBPS_CFG1 (0x41)
#define MCP_16MHz_250kBPS_CFG2 (0xF1)
#define MCP_16MHz_250kBPS_CFG3 (0x85)

#define MCP_16MHz_200kBPS_CFG1 (0x01)
#define MCP_16MHz_200kBPS_CFG2 (0xFA)
#define MCP_16MHz_200kBPS_CFG3 (0x87)

#define MCP_16MHz_125kBPS_CFG1 (0x03)
#define MCP_16MHz_125kBPS_CFG2 (0xF0)
#define MCP_16MHz_125kBPS_CFG3 (0x86)

#define MCP_16MHz_100kBPS_CFG1 (0x03)
#define MCP_16MHz_100kBPS_CFG2 (0xFA)
#define MCP_16MHz_100kBPS_CFG3 (0x87)

#define MCP_16MHz_95kBPS_CFG1 (0x03)
#define MCP_16MHz_95kBPS_CFG2 (0xAD)
#define MCP_16MHz_95kBPS_CFG3 (0x07)

#define MCP_16MHz_83k3BPS_CFG1 (0x03)
#define MCP_16MHz_83k3BPS_CFG2 (0xBE)
#define MCP_16MHz_83k3BPS_CFG3 (0x07)

#define MCP_16MHz_80kBPS_CFG1 (0x03)
#define MCP_16MHz_80kBPS_CFG2 (0xFF)
#define MCP_16MHz_80kBPS_CFG3 (0x87)

#define MCP_16MHz_50kBPS_CFG1 (0x07)
#define MCP_16MHz_50kBPS_CFG2 (0xFA)
#define MCP_16MHz_50kBPS_CFG3 (0x87)

#define MCP_16MHz_40kBPS_CFG1 (0x07)
#define MCP_16MHz_40kBPS_CFG2 (0xFF)
#define MCP_16MHz_40kBPS_CFG3 (0x87)

#define MCP_16MHz_33kBPS_CFG1 (0x09)
#define MCP_16MHz_33kBPS_CFG2 (0xBE)
#define MCP_16MHz_33kBPS_CFG3 (0x07)

#define MCP_16MHz_31k25BPS_CFG1 (0x0F)
#define MCP_16MHz_31k25BPS_CFG2 (0xF1)
#define MCP_16MHz_31k25BPS_CFG3 (0x85)

#define MCP_16MHz_20kBPS_CFG1 (0x0F)
#define MCP_16MHz_20kBPS_CFG2 (0xFF)
#define MCP_16MHz_20kBPS_CFG3 (0x87)

#define MCP_16MHz_10kBPS_CFG1 (0x1F)
#define MCP_16MHz_10kBPS_CFG2 (0xFF)
#define MCP_16MHz_10kBPS_CFG3 (0x87)

#define MCP_16MHz_5kBPS_CFG1 (0x3F)
#define MCP_16MHz_5kBPS_CFG2 (0xFF)
#define MCP_16MHz_5kBPS_CFG3 (0x87)

/*
*  **********************  mcp4728 I2C D/A converter - driver function   ****************************
*/
    void     mcp4728_init();
    void     mcp4728_begin();
    
    uint8_t  mcp4728_setVref(uint8_t, uint8_t, uint8_t, uint8_t);
    void     mcp4728_getStatus();
    uint8_t  mcp4728_writeVref();
    uint8_t  mcp4728_fastWrite();
/*
*  *********************************   mcp2515 driver function   ***********************************
*/
    void  MCP2515_SELECT();
    void  MCP2515_UNSELECT();

    INT8U mcp2515_init(const INT8U canSpeed);                            /* mcp2515init                  */ 
    void  mcp2515_initCANBuffers();
    INT8U mcp2515_setCanSpeed(const INT8U canSpeed);                     /* set Can Speed                */
    INT8U mcp2515_setCANCTRL_Mode(const INT8U newmode);                  /* set mode                     */
    void  mcp2515_setRegister(const INT8U address, const INT8U value);   /* set mcp2515's register       */
    void  mcp2515_setRegisterS(const INT8U address, const INT8U values[], const INT8U n);
    INT8U mcp2515_readRegister(const INT8U address);                     /* read mcp2515's register      */
    void  mcp2515_readRegisterS(const INT8U address, INT8U values[], const INT8U n);
    void  mcp2515_modifyRegister(const INT8U address, const INT8U mask, const INT8U data); /* set bit of one register      */
    void  mcp2515_write_canMsg( const INT8U buffer_sidh_addr );          /* write can msg                */  
    void  mcp2515_start_transmit(const INT8U mcp_addr);                  /* start transmit               */
    INT8U mcp2515_readStatus();                                      /* read mcp2515's Status        */    
    void  mcp2515_read_canMsg( const INT8U buffer_sidh_addr);            /* read can msg                 */
    void  mcp2515_read_id( const INT8U mcp_addr, INT8U* ext, INT32U* id );
    void  mcp2515_write_id( const INT8U mcp_addr, const INT8U ext, const INT32U id );
    void  mcp2515_reset();                                           /* reset mcp2515                */

// ********************************** END mcp2515 driver function ******************************************
/*
* **************************************** CAN operator function  **************************************
*/    
    INT8U CanBegin(INT8U speedset);                                    /* init can                     */
    INT8U CanSetMsg(INT32U id, INT8U ext, INT8U len, INT8U *pData);    /* set message                  */  
    bool  CanSendMsg(INT8U txbuf);                                     /* send message                 */
    INT8U CanReadMsg(BUSTER_CanRxb RxbBuffer);                                                /* read message                 */
    
/////////// DEFINE THE PIN OF THE ARDUINO ///////////
#define LDAC_PIN       22
#define CAN_SPI_CS_PIN 9  

#define spi_readwrite SPI.transfer
#define spi_read() spi_readwrite(0x00)


    // MCP2515 PRIVATE FIELD
static   INT8U   m_nExtFlg;                                                  /* identifier xxxID             *//* or standard (the 11 LSB)     */ /* either extended (the 29 LSB) */                                                            
static   INT32U  m_nID;                                                      /* can id                       */
static   INT8U   m_nDlc;                                                     /* data length:                 */
static   INT8U   m_nDta[MAX_CHAR_IN_MESSAGE];                               /* data                         */
static   INT8U   m_nRtr;                                                     /* rtr                          */
static   INT8U   m_nfilhit;

    // MCP4728 PRIVATE FIELD
static    uint8_t      _dev_address;
static    uint8_t      _deviceID;
static    uint8_t      _intVref[4];
static    uint8_t      _powerDown[4];
static    uint8_t      _gain[4];
static    uint16_t     _values[4];
static    uint16_t     _vOut[4];

void MCP2515_SELECT()
{   
  digitalWrite(CAN_SPI_CS_PIN, LOW); 
}

void MCP2515_UNSELECT()
{
  digitalWrite(CAN_SPI_CS_PIN, HIGH); 
}
/*********************************************************************************************************
** Function name:           mcp2515_reset
** Descriptions:            reset the device
*********************************************************************************************************/
void mcp2515_reset()
{
    MCP2515_SELECT();
    spi_readwrite(MCP_RESET);
    MCP2515_UNSELECT();
    delay(10);
}

/*********************************************************************************************************
** Function name:           mcp2515_readRegister
** Descriptions:            read register
*********************************************************************************************************/
INT8U mcp2515_readRegister(const INT8U address)                                                                     
{
    INT8U ret;
    MCP2515_SELECT();
    spi_readwrite(MCP_READ);
    spi_readwrite(address);
    ret = spi_read();
    MCP2515_UNSELECT();

    return ret;
}

/*********************************************************************************************************
** Function name:           mcp2515_readRegisterS
** Descriptions:            read registerS
*********************************************************************************************************/
void mcp2515_readRegisterS(const INT8U address, INT8U values[], const INT8U n)
{
    INT8U i;
    MCP2515_SELECT();
    spi_readwrite(MCP_READ);
    spi_readwrite(address);
    // mcp2515 has auto-increment of address-pointer
    for (i=0; i<n && i<CAN_MAX_CHAR_IN_MESSAGE; i++) {
        values[i] = spi_read();
    }
    MCP2515_UNSELECT();
}

/*********************************************************************************************************
** Function name:           mcp2515_setRegister
** Descriptions:            set register
*********************************************************************************************************/
void mcp2515_setRegister(const INT8U address, const INT8U value)
{
    MCP2515_SELECT();
    spi_readwrite(MCP_WRITE);
    spi_readwrite(address);
    spi_readwrite(value);
    MCP2515_UNSELECT();
}

/*********************************************************************************************************
** Function name:           mcp2515_setRegisterS
** Descriptions:            set registerS
*********************************************************************************************************/
void mcp2515_setRegisterS(const INT8U address, const INT8U values[], const INT8U n)
{
    INT8U i;
    MCP2515_SELECT();
    spi_readwrite(MCP_WRITE);
    spi_readwrite(address);
       
    for (i=0; i<n; i++) 
    {
        spi_readwrite(values[i]);
    }
    MCP2515_UNSELECT();
}

/*********************************************************************************************************
** Function name:           mcp2515_modifyRegister
** Descriptions:            set bit of one register
*********************************************************************************************************/
void mcp2515_modifyRegister(const INT8U address, const INT8U mask, const INT8U data)
{
    MCP2515_SELECT();
    spi_readwrite(MCP_BITMOD);
    spi_readwrite(address);
    spi_readwrite(mask);
    spi_readwrite(data);
    MCP2515_UNSELECT();
}

/*********************************************************************************************************
** Function name:           mcp2515_readStatus
** Descriptions:            read mcp2515's Status
*********************************************************************************************************/
INT8U mcp2515_readStatus()                             
{
    INT8U i;
    MCP2515_SELECT();
    spi_readwrite(MCP_READ_STATUS);
    i = spi_read();
    MCP2515_UNSELECT();
    
    return i;
}

/*********************************************************************************************************
** Function name:           mcp2515_setCANCTRL_Mode
** Descriptions:            set control mode
*********************************************************************************************************/
INT8U mcp2515_setCANCTRL_Mode(const INT8U newmode)
{
    INT8U i;

    mcp2515_modifyRegister(MCP_CANCTRL, MODE_MASK, newmode);

    i = mcp2515_readRegister(MCP_CANCTRL);
    i &= MODE_MASK;

    if ( i == newmode ) 
    {
        return MCP2515_OK;
    }

    return MCP2515_FAIL;

}

/*********************************************************************************************************
** Function name:           mcp2515_setCanSpeed
** Descriptions:            set boadrate
*********************************************************************************************************/
INT8U mcp2515_setCanSpeed(const INT8U canSpeed)            
{
    INT8U set, cfg1, cfg2, cfg3;
    set = 1;
    switch (canSpeed) 
    {
        case (BUSTER_CAN_5KBPS):
        cfg1 = MCP_16MHz_5kBPS_CFG1;
        cfg2 = MCP_16MHz_5kBPS_CFG2;
        cfg3 = MCP_16MHz_5kBPS_CFG3;
        break;

        case (BUSTER_CAN_10KBPS):
        cfg1 = MCP_16MHz_10kBPS_CFG1;
        cfg2 = MCP_16MHz_10kBPS_CFG2;
        cfg3 = MCP_16MHz_10kBPS_CFG3;
        break;

        case (BUSTER_CAN_20KBPS):
        cfg1 = MCP_16MHz_20kBPS_CFG1;
        cfg2 = MCP_16MHz_20kBPS_CFG2;
        cfg3 = MCP_16MHz_20kBPS_CFG3;
        break;
        
        case (BUSTER_CAN_31K25BPS):
        cfg1 = MCP_16MHz_31k25BPS_CFG1;
        cfg2 = MCP_16MHz_31k25BPS_CFG2;
        cfg3 = MCP_16MHz_31k25BPS_CFG3;
        break;

        case (BUSTER_CAN_33KBPS):
        cfg1 = MCP_16MHz_33kBPS_CFG1;
        cfg2 = MCP_16MHz_33kBPS_CFG2;
        cfg3 = MCP_16MHz_33kBPS_CFG3;
        break;

        case (BUSTER_CAN_40KBPS):
        cfg1 = MCP_16MHz_40kBPS_CFG1;
        cfg2 = MCP_16MHz_40kBPS_CFG2;
        cfg3 = MCP_16MHz_40kBPS_CFG3;
        break;

        case (BUSTER_CAN_50KBPS):
        cfg1 = MCP_16MHz_50kBPS_CFG1;
        cfg2 = MCP_16MHz_50kBPS_CFG2;
        cfg3 = MCP_16MHz_50kBPS_CFG3;
        break;

        case (BUSTER_CAN_80KBPS):
        cfg1 = MCP_16MHz_80kBPS_CFG1;
        cfg2 = MCP_16MHz_80kBPS_CFG2;
        cfg3 = MCP_16MHz_80kBPS_CFG3;
        break;

        case (BUSTER_CAN_83K3BPS):
        cfg1 = MCP_16MHz_83k3BPS_CFG1;
        cfg2 = MCP_16MHz_83k3BPS_CFG2;
        cfg3 = MCP_16MHz_83k3BPS_CFG3;
        break;  

        case (BUSTER_CAN_95KBPS):
        cfg1 = MCP_16MHz_95kBPS_CFG1;
        cfg2 = MCP_16MHz_95kBPS_CFG2;
        cfg3 = MCP_16MHz_95kBPS_CFG3;
        break;

        case (BUSTER_CAN_100KBPS):                                             
        cfg1 = MCP_16MHz_100kBPS_CFG1;
        cfg2 = MCP_16MHz_100kBPS_CFG2;
        cfg3 = MCP_16MHz_100kBPS_CFG3;
        break;

        case (BUSTER_CAN_125KBPS):
        cfg1 = MCP_16MHz_125kBPS_CFG1;
        cfg2 = MCP_16MHz_125kBPS_CFG2;
        cfg3 = MCP_16MHz_125kBPS_CFG3;
        break;

        case (BUSTER_CAN_200KBPS):
        cfg1 = MCP_16MHz_200kBPS_CFG1;
        cfg2 = MCP_16MHz_200kBPS_CFG2;
        cfg3 = MCP_16MHz_200kBPS_CFG3;
        break;

        case (BUSTER_CAN_250KBPS):
        cfg1 = MCP_16MHz_250kBPS_CFG1;
        cfg2 = MCP_16MHz_250kBPS_CFG2;
        cfg3 = MCP_16MHz_250kBPS_CFG3;
        break;

        case (BUSTER_CAN_500KBPS):
        cfg1 = MCP_16MHz_500kBPS_CFG1;
        cfg2 = MCP_16MHz_500kBPS_CFG2;
        cfg3 = MCP_16MHz_500kBPS_CFG3;
        break;
        
        case (BUSTER_CAN_1MBPS):
        cfg1 = MCP_16MHz_1000kBPS_CFG1;
        cfg2 = MCP_16MHz_1000kBPS_CFG2;
        cfg3 = MCP_16MHz_1000kBPS_CFG3;
        break;  

        default:
        set = 0;
        break;
    }

    if (set) {
        mcp2515_setRegister(MCP_CNF1, cfg1);
        mcp2515_setRegister(MCP_CNF2, cfg2);
        mcp2515_setRegister(MCP_CNF3, cfg3);
        return MCP2515_OK;
    }
    else {
        return MCP2515_FAIL;
    }
}

/*********************************************************************************************************
** Function name:           mcp2515_initCANBuffers
** Descriptions:            init canbuffers
*********************************************************************************************************/
void mcp2515_initCANBuffers()
{
    INT8U i, a1, a2, a3;

    a1 = MCP_TXB0CTRL;
    a2 = MCP_TXB1CTRL;
    a3 = MCP_TXB2CTRL;
    for (i = 0; i < 14; i++) {                                          /* in-buffer loop               */
        mcp2515_setRegister(a1, 0);
        mcp2515_setRegister(a2, 0);
        mcp2515_setRegister(a3, 0);
        a1++;
        a2++;
        a3++;
    }
    mcp2515_setRegister(MCP_RXB0CTRL, 0);
    mcp2515_setRegister(MCP_RXB1CTRL, 0);
}

/*********************************************************************************************************
** Function name:           mcp2515_init
** Descriptions:            init the device
*********************************************************************************************************/
INT8U mcp2515_init(const INT8U canSpeed)                       /* mcp2515init                  */
{
    INT8U res;

    mcp2515_reset();

    res = mcp2515_setCANCTRL_Mode(MODE_CONFIG);
    if(res > 0)
    {
      delay(10);
      return res;
    }
    delay(10);
                                                                        /* set boadrate                 */
    if(mcp2515_setCanSpeed(canSpeed))
    {
      delay(10);
      return res;
    }
    delay(10);
    if ( res == MCP2515_OK ) {
                                                                        /* init canbuffers              */
        mcp2515_initCANBuffers();                                       /* interrupt mode               */
        mcp2515_setRegister(MCP_CANINTE, MCP_RX0IF | MCP_RX1IF);

        /* enable both receive-buffers to receive messages  with std. and ext. identifie rs  and enable rollover   */
        mcp2515_modifyRegister(MCP_RXB0CTRL,
          MCP_RXB_RX_MASK | MCP_RXB_BUKT_MASK,
          0x20 /*MCP_RXB_RX_STDEXT | MCP_RXB_BUKT_MASK */ );
        mcp2515_modifyRegister(MCP_RXB1CTRL, MCP_RXB_RX_MASK,
          0x20 /*MCP_RXB_RX_MASK | MCP_RXB_RX_STDEXT */);
                                                                        /* enter normal mode            */
        delay(10);
    }
    return res;
}

/*********************************************************************************************************
** Function name:           mcp2515_write_id
** Descriptions:            write can id
*********************************************************************************************************/
void mcp2515_write_id( const INT8U mcp_addr, const INT8U ext, const INT32U id )
{
    uint16_t canid;
    INT8U tbufdata[4];

    canid = (uint16_t)(id & 0x0FFFF);

    if ( ext == 1) 
    {
        tbufdata[MCP_EID0] = (INT8U) (canid & 0xFF);
        tbufdata[MCP_EID8] = (INT8U) (canid >> 8);
        canid = (uint16_t)(id >> 16);
        tbufdata[MCP_SIDL] = (INT8U) (canid & 0x03);
        tbufdata[MCP_SIDL] += (INT8U) ((canid & 0x1C) << 3);
        tbufdata[MCP_SIDL] |= MCP_TXB_EXIDE_M;
        tbufdata[MCP_SIDH] = (INT8U) (canid >> 5 );
    }
    else 
    {
        tbufdata[MCP_SIDH] = (INT8U) (canid >> 3 );
        tbufdata[MCP_SIDL] = (INT8U) ((canid & 0x07 ) << 5);
        tbufdata[MCP_EID0] = 0;
        tbufdata[MCP_EID8] = 0;
    }
    mcp2515_setRegisterS( mcp_addr, tbufdata, 4 );
}

/*********************************************************************************************************
** Function name:           mcp2515_read_id
** Descriptions:            read can id
*********************************************************************************************************/
void mcp2515_read_id( const INT8U mcp_addr, INT8U* ext, INT32U* id )
{
    INT8U tbufdata[4];

    *ext = 0;
    *id = 0;

    mcp2515_readRegisterS( mcp_addr, tbufdata, 4 );

    *id = (tbufdata[MCP_SIDH]<<3) + (tbufdata[MCP_SIDL]>>5);

    if ( (tbufdata[MCP_SIDL] & MCP_TXB_EXIDE_M) ==  MCP_TXB_EXIDE_M ) 
    {
                                                                        /* extended id                  */
        *id = (*id<<2) + (tbufdata[MCP_SIDL] & 0x03);
        *id = (*id<<8) + tbufdata[MCP_EID8];
        *id = (*id<<8) + tbufdata[MCP_EID0];
        *ext = 1;
    }
}

/*********************************************************************************************************
** Function name:           mcp2515_write_canMsg
** Descriptions:            write msg
*********************************************************************************************************/
void mcp2515_write_canMsg( const INT8U buffer_sidh_addr)
{
    INT8U mcp_addr;
    mcp_addr = buffer_sidh_addr;
    mcp2515_setRegisterS(mcp_addr+5, m_nDta, m_nDlc );                  /* write data bytes             */
    if ( m_nRtr == 1)                                                   /* if RTR set bit in byte       */
    {
        m_nDlc |= MCP_RTR_MASK;  
    }
    mcp2515_setRegister((mcp_addr+4), m_nDlc );                        /* write the RTR and DLC        */
    mcp2515_write_id(mcp_addr, m_nExtFlg, m_nID );                     /* write CAN id                 */

}

/*********************************************************************************************************
** Function name:           mcp2515_read_canMsg
** Descriptions:            read message
*********************************************************************************************************/
void mcp2515_read_canMsg( const INT8U buffer_sidh_addr)        /* read can msg                 */
{
    INT8U mcp_addr, ctrl;

    mcp_addr = buffer_sidh_addr;

    mcp2515_read_id( mcp_addr, &m_nExtFlg,&m_nID );

    ctrl = mcp2515_readRegister( mcp_addr-1 );
    m_nDlc = mcp2515_readRegister( mcp_addr+4 );

    if ((ctrl & 0x08)) {
        m_nRtr = 1;
    }
    else {
        m_nRtr = 0;
    }

    m_nDlc &= MCP_DLC_MASK;
    mcp2515_readRegisterS( mcp_addr+5, &(m_nDta[0]), m_nDlc );
}

/*********************************************************************************************************
** Function name:           sendMsg
** Descriptions:            send message
*********************************************************************************************************/
void mcp2515_start_transmit(const INT8U mcp_addr)              /* start transmit               */
{
    mcp2515_modifyRegister( mcp_addr-1 , MCP_TXB_TXREQ_M, MCP_TXB_TXREQ_M );
}

/*********************************************************************************************************
** Function name:           init
** Descriptions:            init can and set speed
*********************************************************************************************************/
INT8U CanBegin(INT8U speedset)
{
  INT8U res;
  
  SPI.begin();
  //SPI.setDataMode(SPI_MODE1);
  res = mcp2515_init(speedset);
  if (res == MCP2515_OK) 
    return CAN_OK;
  else 
    return CAN_FAILINIT;
}


/*********************************************************************************************************
** Function name:           CanSetMsg
** Descriptions:            set can message, such as dlc, id, dta[] and so on
*********************************************************************************************************/
INT8U CanSetMsg(INT32U id, INT8U ext, INT8U len, INT8U *pData)
{
    int i = 0;
    m_nExtFlg = ext;
    m_nID     = id;
    m_nDlc    = len;
    for(i = 0; i<MAX_CHAR_IN_MESSAGE; i++)
    {
        m_nDta[i] = *(pData+i);
    }
    return MCP2515_OK;
}

/*********************************************************************************************************
** Function name:           CanReadMsg
** Parameter RxbBuffer:     Read only the message in the RxbBuffer
** Descriptions:            read message
*********************************************************************************************************/
INT8U CanReadMsg(BUSTER_CanRxb RxbBuffer)
{
    INT8U stat, res;

    stat = mcp2515_readStatus();

    if (( stat & MCP_STAT_RX0IF ) && (RxbBuffer == BUSTER_Rxb0))                                        /* Msg in Buffer 0              */
    {
        mcp2515_read_canMsg( MCP_RXBUF_0);
        mcp2515_modifyRegister(MCP_CANINTF, MCP_RX0IF, 0);
        res = CAN_OK;
    }
    else if (( stat & MCP_STAT_RX1IF ) && (RxbBuffer == BUSTER_Rxb1))                                  /* Msg in Buffer 1              */
    {
        mcp2515_read_canMsg( MCP_RXBUF_1);
        mcp2515_modifyRegister(MCP_CANINTF, MCP_RX1IF, 0);
        res = CAN_OK;
    }
    else 
    {
        res = CAN_NOMSG;
    }
    return res;
}

/*********************************************************************************************************
** Function name:           CanSendMsg
** Descriptions:            send message
*********************************************************************************************************/

bool CanSendMsg(INT8U txbuf)
{
    INT8U res, res1, txbuf_n;
    uint16_t uiTimeOut = 0;
    
    txbuf_n = txbuf + 1;
    
    mcp2515_write_canMsg( txbuf_n);
    
    mcp2515_start_transmit( txbuf_n );
    
    return true;
}

//**************************************************************************************************
//************************************** MCP 4728 FUNCTIONS ****************************************
//**************************************************************************************************
/*
Init the I2C D/A converter -  mcp4728
*/
void mcp4728_init()
{
  _deviceID = 0x00;
  _dev_address = (BASE_ADDR | _deviceID);
  mcp4728_begin();  // initialize i2c interface  
}


/*
Begin I2C, get current values (input register and eeprom) of mcp4728
*/
void mcp4728_begin()
{
  Wire.begin();
  Wire.setClock(400000L);
  mcp4728_getStatus();
  mcp4728_setVref(0,0,0,0);
  _powerDown[0] = _powerDown[1] = _powerDown[2] = _powerDown[3] = 0;
}
/* 
  Write Voltage reference settings to input regiters
    Vref setting = 1 (internal), Gain = 0 (x1)   ==> Vref = 2.048V
    Vref setting = 1 (internal), Gain = 1 (x2)   ==> Vref = 4.096V
    Vref setting = 0 (external), Gain = ignored  ==> Vref = VDD
*/
uint8_t mcp4728_setVref(uint8_t value1, uint8_t value2, uint8_t value3, uint8_t value4) {
  _intVref[0] = value1;
  _intVref[1] = value2;
  _intVref[2] = value3;
  _intVref[3] = value4;
  return mcp4728_writeVref();
}

/*
Get current values (input register and eeprom) of mcp4728
*/
void mcp4728_getStatus()
{
  Wire.requestFrom(int(_dev_address), 24);
  while(Wire.available())
  {
    int deviceID = Wire.read();
    int hiByte = Wire.read();
    int loByte = Wire.read();

    int isEEPROM = (deviceID & 0B00001000) >> 3;
    int channel = (deviceID & 0B00110000) >> 4;
    if (isEEPROM != 1) {
      _intVref[channel] = (hiByte & 0B10000000) >> 7;
      _gain[channel] = (hiByte & 0B00010000) >> 4;
      _values[channel] = word((hiByte & 0B00001111), loByte);
    }
  }
}
/*
Write Voltage reference setting to input registers. refer to DATASHEET 5.6.5
No EEPROM update
*/
uint8_t mcp4728_writeVref() {
  Wire.write(VREFWRITE | _intVref[0] << 3 | _intVref[1] << 2 | _intVref[2] << 1 | _intVref[3]); 
}



/*
FastWrite input register values - All DAC ouput update. refer to DATASHEET 5.6.1
DAC Input and PowerDown bits update.
No EEPROM update
*/
uint8_t mcp4728_fastWrite() {

  Wire.beginTransmission(_dev_address);
  for (uint8_t channel=0; channel <= 3; channel++) {
    Wire.write(highByte(_values[channel]));
    Wire.write(lowByte(_values[channel]));
  }
  return Wire.endTransmission();
}


const unsigned char PS_16 = (1 << ADPS2);
const unsigned char PS_32 = (1 << ADPS2) | (1 << ADPS0);
const unsigned char PS_64 = (1 << ADPS2) | (1 << ADPS1);
const unsigned char PS_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);


// Analog inputs
#define BUSTER_PIN_A_IN_0    15
#define BUSTER_PIN_A_IN_1    14
#define BUSTER_PIN_A_IN_2    13
#define BUSTER_PIN_A_IN_3    12

void BusTer_InitInputOutput(){
  // set up the ADC for higher rate
  ADCSRA &= ~PS_128;  // remove bits set by Arduino library
  // you can choose a prescaler from above.
  // PS_16, PS_32, PS_64 or PS_128
  ADCSRA |= PS_16;    // set our own prescaler to 16

  // serial setup
  Serial.begin(115200);
  
  mcp4728_init(); // analog output
  // If LDAC pin is not grounded, need to be pull down for normal operation.
  pinMode(LDAC_PIN, OUTPUT); 
  digitalWrite(LDAC_PIN, LOW);
  
  // INPUT /  OUTPUT setup
  pinMode(BUSTER_PIN_OUT_0, OUTPUT);
  pinMode(BUSTER_PIN_OUT_1, OUTPUT);
  pinMode(BUSTER_PIN_OUT_2, OUTPUT);
  pinMode(BUSTER_PIN_OUT_3, OUTPUT);
  pinMode(BUSTER_PIN_OUT_4, OUTPUT);
  pinMode(BUSTER_PIN_OUT_5, OUTPUT);
  pinMode(BUSTER_PIN_OUT_6, OUTPUT);
  pinMode(BUSTER_PIN_OUT_7, OUTPUT);
  
  pinMode(BUSTER_PIN_STATUS_LED_GREEN, OUTPUT);
  pinMode(BUSTER_PIN_STATUS_LED_RED, OUTPUT);
  
  pinMode(BUSTER_PIN_IN_0, INPUT);
  pinMode(BUSTER_PIN_IN_1, INPUT);
  pinMode(BUSTER_PIN_IN_2, INPUT);
  pinMode(BUSTER_PIN_IN_3, INPUT);
  pinMode(BUSTER_PIN_IN_4, INPUT);
  pinMode(BUSTER_PIN_IN_5, INPUT);
  pinMode(BUSTER_PIN_IN_6, INPUT);
  pinMode(BUSTER_PIN_IN_7, INPUT);

  pinMode(BUSTER_PIN_DIPSWITCH_1, INPUT);  
  pinMode(BUSTER_PIN_DIPSWITCH_2, INPUT); 
  pinMode(BUSTER_PIN_DIPSWITCH_3, INPUT); 
  pinMode(BUSTER_PIN_DIPSWITCH_4, INPUT); 
  pinMode(BUSTER_PIN_DIPSWITCH_5, INPUT); 
  pinMode(BUSTER_PIN_DIPSWITCH_6, INPUT); 
  pinMode(BUSTER_PIN_DIPSWITCH_7, INPUT); 
  pinMode(BUSTER_PIN_DIPSWITCH_8, INPUT); 
}

const float ratio = 5000.0f / 1023; // 5 V = 1023 (2^10 - 1, 10 bits)

int  BusTer_AnalogRead(BUSTER_AnalogInput input)
{
  int readValue;
  switch(input){
    case BUSTER_ANALOGINPUT_0 :
        readValue = analogRead(BUSTER_PIN_A_IN_0);
        break;
    case BUSTER_ANALOGINPUT_1 : 
        readValue = analogRead(BUSTER_PIN_A_IN_1);
        break;
    case BUSTER_ANALOGINPUT_2 : 
        readValue = analogRead(BUSTER_PIN_A_IN_2);
        break;
    case BUSTER_ANALOGINPUT_3 : 
        readValue = analogRead(BUSTER_PIN_A_IN_3);
        break;  
  }
  return readValue * ratio;
}

const float  outputRatio = 4095.0f / defaultVDD;

void BusTer_AnalogWrite(int output0, int output1, int output2, int output3)
{
  if(output0 >= defaultVDD)
    output0 = defaultVDD-1;
  if(output1 >= defaultVDD)
    output1 = defaultVDD-1;
  if(output2 >= defaultVDD)
    output2 = defaultVDD-1;
  if(output3 >= defaultVDD)
    output3 = defaultVDD-1;
  
  _values[0] = output0 * outputRatio;
  _values[1] = output1 * outputRatio;
  _values[2] = output2 * outputRatio;
  _values[3] = output3 * outputRatio;

  mcp4728_fastWrite();
}

void BusTer_CanInit(BUSTER_CanSpeed canSpeed)
{   
    pinMode(CAN_SPI_CS_PIN, OUTPUT);
    MCP2515_UNSELECT();
    delay(100);
    SPI.setClockDivider(SPI_CLOCK_DIV2);
    
    while(CAN_OK != CanBegin(canSpeed)) // init can bus : baudrate = 500k
    {
        Serial.println("CAN BUS Shield init fail");
        Serial.println("Init CAN BUS Shield again");
        delay(100);
    }
}

bool BusTer_CanSend(BUSTER_CanTxb Txbuf, BUSTER_CanType type,  INT32U arb, INT8U len, INT8U *buf)
{     
      CanSetMsg(arb, type, len, buf);
      return CanSendMsg(Txbuf);
    
}

bool BusTer_CanSendReady(BUSTER_CanTxb txbuf_n)                 /* get Next free txbuf          */
{
  INT8U ctrlval = mcp2515_readRegister( txbuf_n );
  if ( (ctrlval & MCP_TXB_TXREQ_M) == 0 )
    return true;
  return false;          
}

bool BusTer_CanSetMask(BUSTER_CanRxm rxMask, BUSTER_CanType type, INT32U ulData)
{
    INT8U res = MCP2515_OK;
    res = mcp2515_setCANCTRL_Mode(MODE_CONFIG);
    if(res > 0){
      return false;
    }

    if (rxMask == BUSTER_Rxm0){
        mcp2515_write_id(MCP_RXM0SIDH, type, ulData);

    }
    else if(rxMask == BUSTER_Rxm1){
        mcp2515_write_id(MCP_RXM1SIDH, type, ulData);
    }
    else res =  MCP2515_FAIL;

    return res;
}

bool BusTer_CanSetFilter(BUSTER_CanRxf rxFilter, BUSTER_CanType type, INT32U ulData)
{
    INT8U res = MCP2515_OK;
    res = mcp2515_setCANCTRL_Mode(MODE_CONFIG);
    if(res > 0)
    {
      return false;
    }
    
    switch( rxFilter )
    {
        case BUSTER_Rxf0:
        mcp2515_write_id(MCP_RXF0SIDH, type, ulData);
        break;

        case BUSTER_Rxf1:
        mcp2515_write_id(MCP_RXF1SIDH, type, ulData);
        break;

        case BUSTER_Rxf2:
        mcp2515_write_id(MCP_RXF2SIDH, type, ulData);
        break;

        case BUSTER_Rxf3:
        mcp2515_write_id(MCP_RXF3SIDH, type, ulData);
        break;

        case BUSTER_Rxf4:
        mcp2515_write_id(MCP_RXF4SIDH, type, ulData);
        break;

        case BUSTER_Rxf5:
        mcp2515_write_id(MCP_RXF5SIDH, type, ulData);
        break;

        default:
        res = MCP2515_FAIL;
    }
    return res;
}

bool BusTer_CanActivateNormalMode()
{
    mcp2515_setCANCTRL_Mode(MODE_NORMAL);
}

/*********************************************************************************************************
** Function name:           BusTer_CanMessageReceived
** parameter RxbBuffer :    buffer to check if a message is received
** Descriptions:            check if got something, return true if a message arrived
*********************************************************************************************************/
bool BusTer_CanMessageReceived(BUSTER_CanRxb RxbBuffer)
{
    INT8U res = mcp2515_readStatus();                                         /* RXnIF in Bit 1 and 0         */
    if ( res & MCP_STAT_RXIF_MASK ) 
    {
      switch(RxbBuffer){
        case BUSTER_Rxb0 :
          if(res & MCP_STAT_RX0IF)
            return true;
          else 
            return false;
          break;
        case BUSTER_Rxb1 :
          if(res & MCP_STAT_RX1IF)
            return true;
          else 
            return false;
          break;
        default: 
          return false;
      }
    }
    else 
    {
        return false;
    }
}

/*********************************************************************************************************
** Function name:           BusTer_CanRead
** parameter arb :          return the arbitration field
** parameter len :          return the lenght of the data buffer
** parameter buf[] :        adress of the buffer, stock the data into
** Descriptions:            reads message buf and can bus source arb, returns true if success
*********************************************************************************************************/
bool BusTer_CanRead(BUSTER_CanRxb RxbBuffer, INT32U *arb, INT8U *len, INT8U buf[])
{
    INT8U rc = CanReadMsg(RxbBuffer);
    if (rc == CAN_OK) {
       *len = m_nDlc;
       *arb  = m_nID;
       for(int i = 0; i<m_nDlc && i < MAX_CHAR_IN_MESSAGE; i++) {
          buf[i] = m_nDta[i];
       }
       return true;
    } else {
       *len = 0;
       return false;
    }
}

/*********************************************************************************************************
** Function name:           BusTer_CanHasError
** Descriptions:            returns true when a can error occurs
*********************************************************************************************************/
bool BusTer_CanHasError()
{
    INT8U eflg = mcp2515_readRegister(MCP_EFLG);
    if ( eflg & MCP_EFLG_ERRORMASK ) 
    {
        return true;
    }
    else 
    {
        return false;
    }
}

/*********************************************************************************************************
** Function name:           BusTer_CanClearError
** Descriptions:            set all message to zero and clear can error
*********************************************************************************************************/
void BusTer_CanClearError()
{
    m_nID       = 0;
    m_nDlc      = 0;
    m_nExtFlg   = 0;
    m_nRtr      = 0;
    m_nfilhit   = 0;
    for(int i = 0; i<m_nDlc; i++ )
      m_nDta[i] = 0x00;
    
    mcp2515_setRegister(MCP_EFLG, 0);
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
