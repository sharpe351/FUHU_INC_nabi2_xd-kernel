
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/timer.h>
#include <linux/gpio.h>

#include <linux/sysfs.h>
#include <linux/init.h>
#include <linux/mutex.h>
#include <mach/gpio.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/slab.h>

#include <linux/syscalls.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/fcntl.h>
#include <linux/string.h>
#include <asm/unistd.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>
#include <linux/earlysuspend.h>
#include <linux/input.h>

#include "../../../arch/arm/mach-tegra/gpio-names.h"

#include <linux/nabijr_board_version.h>

//-------------------------------------------------------------------
//-------------------------------------------------------------------
#define u8         unsigned char
#define u16        unsigned short
#define u32        unsigned int
#define s32        signed int

#define REPORT_PACKET_LENGTH    (8)
#define MSG21XX_INT_GPIO       TEGRA_GPIO_PH4 /*(65)*/
#define MSG21XX_RESET_GPIO    TEGRA_GPIO_PH5 /*(22)*/
#define MS_TS_MSG21XX_X_MAX   (800)/*(320)*/
#define MS_TS_MSG21XX_Y_MAX   (480)

/*20130326, JimmySu add board ID*/
static int touch_board_id =0x1;

static int msg21xx_irq = 0;
static struct i2c_client     *msg21xx_i2c_client;
static struct i2c_client     *this_client;
static struct work_struct    msg21xx_wq;
static struct early_suspend  early_suspend;
static struct input_dev     *input = NULL;

struct msg21xx_ts_data
{
    uint16_t            addr;
    struct i2c_client  *client;
    struct input_dev   *input_dev;
    int                 use_irq;
    struct work_struct  work;
    int ( *power ) ( int on );
    int ( *get_int_status ) ( void );
    void ( *reset_ic ) ( void );
};

#define MAX_TOUCH_FINGER 2
typedef struct
{
    u16 X;
    u16 Y;
} TouchPoint_t;

typedef struct
{
    u8 nTouchKeyMode;
    u8 nTouchKeyCode;
    u8 nFingerNum;
    TouchPoint_t Point[MAX_TOUCH_FINGER];
} TouchScreenInfo_t;

typedef enum
{
    EMEM_ALL = 0,
    EMEM_MAIN,
    EMEM_INFO,
} EMEM_TYPE_t;

typedef enum bool_t
{
    FALSE = 0,
    TRUE  = 1
} bool_t;

typedef struct
{
    u32 loop;
    u32 unit;
} SW_DELAY;

#define MSG2133_UPDATE				1

#ifdef MSG2133_UPDATE
#define MSG2133_SOFTWARE_UPDATE		1
//#define MSG2133_AUTO_UPDATE		1
#define _FW_UPDATE_C3_                     1
//#define MSG2133_DRIVER_UPDATE		1
#endif

/**==========================================================**/
#define TS_DEBUG_MSG 				1



#define MSG2133_BUS_NUM			2
//#define MSG2133_TS_NAME	   	    "msg2133"
#define MSG2133_TS_NAME	   	    "ms-msg21xx"
#define MSG2133_TS_ADDR			0x26

#define MSG2133_FW_ADDR			0x62
#define MSG2133_FW_UPDATE_ADDR   	0x49	

#define FW_ADDR_MSG21XX   (0xC4>>1)
#define FW_ADDR_MSG21XX_TP   (0x4C>>1)
#define FW_UPDATE_ADDR_MSG21XX   (0x92>>1)

#define TPD_OK 					0
#define TPD_REG_BASE 			0x00
#define TPD_SOFT_RESET_MODE 	0x01
#define TPD_OP_MODE 			0x00
#define TPD_LOW_PWR_MODE 		0x04
#define TPD_SYSINFO_MODE 		0x10

#define GET_HSTMODE(reg)  ((reg & 0x70) >> 4)  // in op mode or not 
#define GET_BOOTLOADERMODE(reg) ((reg & 0x10) >> 4)  // in bl mode 

static int tp_type;
static unsigned char g_dwiic_info_data[1024];   // Buffer for info data

#ifdef MSG2133_DRIVER_UPDATE
struct delayed_work delay_work_driver_fw_check;
static u16  fw_version_now ;
#endif

#if TS_DEBUG_MSG
#define MSG2133_DBG(format, ...)	printk(KERN_INFO "MSG2133 " format "\n", ## __VA_ARGS__)
#else
#define MSG2133_DBG(format, ...)
#endif

struct fw_version {
	unsigned short major;
	unsigned short minor;
	unsigned short VenderID;
};

enum {
	TP_HUANGZE = 1, //huangze
	TP_JUNDA,
	TP_MUDOND,
};

#ifdef MSG2133_UPDATE
struct class *firmware_class;
struct device *firmware_cmd_dev;
static int update_switch = 0;
static int FwDataCnt;
static struct fw_version fw_v;
static unsigned char temp[32][1024];

static unsigned char FW_130131[] = {
//	#include "msg_21xx_fw_101.h"
};
static unsigned char FW_PF[] = {
	#include "msg_21xx_fw_104.h"
};
static unsigned char FW_GF[] = {
//	#include "msg_21xx_fw_106.h"
//	#include "msg_21xx_fw_108.h"
	#include "msg_21xx_fw_109.h"	
};


static bool msg2133_i2c_read(char *pbt_buf, int dw_lenth)
{
    int ret;
//    MSG2133_DBG("The msg_i2c_client->addr=0x%x\n",this_client->addr);
    ret = i2c_master_recv(this_client, pbt_buf, dw_lenth);

    if(ret <= 0){
        MSG2133_DBG("msg_i2c_read_interface error\n");
        return false;
    }

    return true;
}

static bool msg2133_i2c_write(char *pbt_buf, int dw_lenth)
{
    int ret;
//    MSG2133_DBG("The msg_i2c_client->addr=0x%x\n",this_client->addr);
    ret = i2c_master_send(this_client, pbt_buf, dw_lenth);

    if(ret <= 0){
        MSG2133_DBG("msg_i2c_read_interface error\n");
        return false;
    }

    return true;
}

static void i2c_read_msg2133(unsigned char *pbt_buf, int dw_lenth)
{
    this_client->addr = MSG2133_FW_ADDR;
	i2c_master_recv(this_client, pbt_buf, dw_lenth);	//0xC5_8bit
	this_client->addr = MSG2133_TS_ADDR;
}

static void i2c_write_msg2133(unsigned char *pbt_buf, int dw_lenth)
{

	this_client->addr = MSG2133_FW_ADDR;
	i2c_master_send(this_client, pbt_buf, dw_lenth);		//0xC4_8bit
	this_client->addr = MSG2133_TS_ADDR;
}

static void i2c_read_update_msg2133(unsigned char *pbt_buf, int dw_lenth)
{	

	this_client->addr = MSG2133_FW_UPDATE_ADDR;
	i2c_master_recv(this_client, pbt_buf, dw_lenth);	//0x93_8bit
	this_client->addr = MSG2133_TS_ADDR;
}

static void i2c_write_update_msg2133(unsigned char *pbt_buf, int dw_lenth)
{	
    this_client->addr = MSG2133_FW_UPDATE_ADDR;
	i2c_master_send(this_client, pbt_buf, dw_lenth);	//0x92_8bit
	this_client->addr = MSG2133_TS_ADDR;
}

void dbbusDWIICEnterSerialDebugMode(void)//Check
{
    unsigned char data[5];
    // Enter the Serial Debug Mode
    data[0] = 0x53;
    data[1] = 0x45;
    data[2] = 0x52;
    data[3] = 0x44;
    data[4] = 0x42;
    i2c_write_msg2133(data, 5);
}

void dbbusDWIICStopMCU(void)//Check
{
    unsigned char data[1];
    // Stop the MCU
    data[0] = 0x37;
    i2c_write_msg2133(data, 1);
}

void dbbusDWIICIICUseBus(void)//Check
{
    unsigned char data[1];
    // IIC Use Bus
    data[0] = 0x35;
    i2c_write_msg2133(data, 1);
}

void dbbusDWIICIICReshape(void)//Check
{
    unsigned char data[1];
    // IIC Re-shape
    data[0] = 0x71;
    i2c_write_msg2133(data, 1);
}

void dbbusDWIICIICNotUseBus(void)//Check
{
    unsigned char data[1];
    // IIC Not Use Bus
    data[0] = 0x34;
    i2c_write_msg2133(data, 1);
}

void dbbusDWIICNotStopMCU(void)//Check
{
    unsigned char data[1];
    // Not Stop the MCU
    data[0] = 0x36;
    i2c_write_msg2133(data, 1);
}

void dbbusDWIICExitSerialDebugMode(void)//Check
{
    unsigned char data[1];
    // Exit the Serial Debug Mode
    data[0] = 0x45;
    i2c_write_msg2133(data, 1);
    // Delay some interval to guard the next transaction
}

void drvISP_EntryIspMode(void)//Check
{
    unsigned char bWriteData[5] =
    {
        0x4D, 0x53, 0x54, 0x41, 0x52
    };
    i2c_write_update_msg2133(bWriteData, 5);
    msleep(10);           // delay about 10ms
}

void drvISP_WriteEnable(void)//Check
{
    unsigned char bWriteData[2] =
    {
        0x10, 0x06
    };
    unsigned char bWriteData1 = 0x12;
    i2c_write_update_msg2133(bWriteData, 2);
    i2c_write_update_msg2133(&bWriteData1, 1);
}

void drvISP_ExitIspMode(void)//Check
{
	unsigned char bWriteData = 0x24;
	i2c_write_update_msg2133(&bWriteData, 1);
}

static unsigned char drvISP_Read(unsigned char n, unsigned char *pDataToRead) //First it needs send 0x11 to notify we want to get flash data back. //Check
{
    unsigned char Read_cmd = 0x11;
    unsigned char dbbus_rx_data[2] = {0xFF, 0xFF};
    i2c_write_update_msg2133(&Read_cmd, 1);
    msleep(10);        // delay about 1000us*****
    if ( n == 1 )
    {
        i2c_read_update_msg2133( &dbbus_rx_data[0], 2 );

        // Ideally, the obtained dbbus_rx_data[0~1] stands for the following meaning:
        //  dbbus_rx_data[0]  |  dbbus_rx_data[1]  | status
        // -------------------+--------------------+--------
        //       0x00         |       0x00         |  0x00
        // -------------------+--------------------+--------
        //       0x??         |       0x00         |  0x??
        // -------------------+--------------------+--------
        //       0x00         |       0x??         |  0x??
        //
        // Therefore, we build this field patch to return the status to *pDataToRead.
        *pDataToRead = ( ( dbbus_rx_data[0] >= dbbus_rx_data[1] ) ? \
                         dbbus_rx_data[0]  : dbbus_rx_data[1] );
    }
    else
    {
        i2c_read_update_msg2133 ( pDataToRead, n );
    }

    return 0;
}


unsigned char drvISP_ReadStatus(void)//Check
{
    unsigned char bReadData = 0;
    unsigned char bWriteData[2] =
    {
        0x10, 0x05
    };
    unsigned char bWriteData1 = 0x12;
//    msleep(1);           // delay about 100us
    i2c_write_update_msg2133(bWriteData, 2);
    msleep(1);           // delay about 100us
    drvISP_Read(1, &bReadData);
//    msleep(10);           // delay about 10ms
    i2c_write_update_msg2133(&bWriteData1, 1);
    return bReadData;
}


void drvISP_SectorErase(unsigned int addr)//This might remove
{
	unsigned char bWriteData[5] = { 0x00, 0x00, 0x00, 0x00, 0x00 };
	unsigned char  bWriteData1 = 0x12;
	printk("The drvISP_ReadStatus0=%d\n", drvISP_ReadStatus());
	drvISP_WriteEnable();
	printk("The drvISP_ReadStatus1=%d\n", drvISP_ReadStatus());
	bWriteData[0] = 0x10;
	bWriteData[1] = 0x50;
	i2c_write_update_msg2133(&bWriteData, 2);
	i2c_write_update_msg2133(&bWriteData1, 1);
	bWriteData[0] = 0x10;
	bWriteData[1] = 0x01;
	bWriteData[2] = 0x00;
	i2c_write_update_msg2133(bWriteData, 3);
	i2c_write_update_msg2133(&bWriteData1, 1);
	bWriteData[0] = 0x10;
	bWriteData[1] = 0x04;
	i2c_write_update_msg2133(bWriteData, 2);
	i2c_write_update_msg2133(&bWriteData1, 1); 
	while((drvISP_ReadStatus() & 0x01) == 0x01);
	
	drvISP_WriteEnable();
	bWriteData[0] = 0x10;
	bWriteData[1] = 0x20; //Sector Erase
	bWriteData[2] = (( addr >> 16) & 0xFF);
	bWriteData[3] = (( addr >> 8 ) & 0xFF);
	bWriteData[4] = ( addr & 0xFF); 
	i2c_write_update_msg2133(&bWriteData, 5);
	i2c_write_update_msg2133(&bWriteData1, 1);
	while((drvISP_ReadStatus() & 0x01) == 0x01);
}

static void drvISP_BlockErase(unsigned int addr)//This might remove
{
    unsigned char bWriteData[5] = { 0x00, 0x00, 0x00, 0x00, 0x00 };
    unsigned char bWriteData1 = 0x12;
    unsigned int timeOutCount=0;
	
    drvISP_WriteEnable();
    //Enable write status register
    bWriteData[0] = 0x10;
    bWriteData[1] = 0x50;
    i2c_write_update_msg2133(bWriteData, 2);
    i2c_write_update_msg2133(&bWriteData1, 1);
    //Write Status
    bWriteData[0] = 0x10;
    bWriteData[1] = 0x01;
    bWriteData[2] = 0x00;
    i2c_write_update_msg2133(bWriteData, 3);
    i2c_write_update_msg2133(&bWriteData1, 1);
    //Write disable
    bWriteData[0] = 0x10;
    bWriteData[1] = 0x04;
    i2c_write_update_msg2133(bWriteData, 2);
    i2c_write_update_msg2133(&bWriteData1, 1);

    timeOutCount=0;
    msleep(1);           // delay about 100us
    while((drvISP_ReadStatus() & 0x01) == 0x01)
    {
        timeOutCount++;
	 if ( timeOutCount > 10000 ) 
            break; /* around 1 sec timeout */
    }

    //pr_ch("The drvISP_ReadStatus3=%d\n", drvISP_ReadStatus());
    drvISP_WriteEnable();
    //pr_ch("The drvISP_ReadStatus4=%d\n", drvISP_ReadStatus());
    bWriteData[0] = 0x10;
    bWriteData[1] = 0xC7;        //Block Erase
    //bWriteData[2] = ((addr >> 16) & 0xFF) ;
    //bWriteData[3] = ((addr >> 8) & 0xFF) ;
    // bWriteData[4] = (addr & 0xFF) ;
    i2c_write_update_msg2133(bWriteData, 2);
    //i2c_write_update_msg2133( &bWriteData, 5);
    i2c_write_update_msg2133(&bWriteData1, 1);

    timeOutCount=0;
    msleep(1);           // delay about 100us
    while((drvISP_ReadStatus() & 0x01) == 0x01)
    {
        timeOutCount++;
	 if ( timeOutCount > 10000 ) 
            break; /* around 1 sec timeout */
    }
}

static void drvISP_ChipErase()//new, check
{
    unsigned char bWriteData[5] = { 0x00, 0x00, 0x00, 0x00, 0x00 };
    unsigned char bWriteData1 = 0x12;
    unsigned int timeOutCount = 0;
    drvISP_WriteEnable();

    //Enable write status register
    bWriteData[0] = 0x10;
    bWriteData[1] = 0x50;
    i2c_write_update_msg2133 ( bWriteData, 2 );
    i2c_write_update_msg2133 ( &bWriteData1, 1 );

    //Write Status
    bWriteData[0] = 0x10;
    bWriteData[1] = 0x01;
    bWriteData[2] = 0x00;
    i2c_write_update_msg2133 ( bWriteData, 3 );
    i2c_write_update_msg2133 ( &bWriteData1, 1 );

    //Write disable
    bWriteData[0] = 0x10;
    bWriteData[1] = 0x04;
    i2c_write_update_msg2133 ( bWriteData, 2 );
    i2c_write_update_msg2133 ( &bWriteData1, 1 );
    msleep(1);        // delay about 100us*****
    timeOutCount = 0;
    while ( ( drvISP_ReadStatus() & 0x01 ) == 0x01 )
    {
        timeOutCount++;
        if ( timeOutCount >= 100000 ) break; /* around 1 sec timeout */
    }
    drvISP_WriteEnable();

    bWriteData[0] = 0x10;
    bWriteData[1] = 0xC7;

    i2c_write_update_msg2133 ( bWriteData, 2 );
    i2c_write_update_msg2133 ( &bWriteData1, 1 );
    msleep(1);        // delay about 100us*****
    timeOutCount = 0;
    while ( ( drvISP_ReadStatus() & 0x01 ) == 0x01 )
    {
        timeOutCount++;
        if ( timeOutCount >= 500000 ) break; /* around 5 sec timeout */
    }
}



void drvISP_Program(unsigned short k, unsigned char *pDataToWrite)//Check
{
    unsigned short i = 0;
    unsigned short j = 0;
    //U16 n = 0;
    unsigned char TX_data[133];
    unsigned char bWriteData1 = 0x12;
    unsigned int addr = k * 1024;
    unsigned int timeOutCount = 0; 

    for(j = 0; j < 8; j++)    //128*8 cycle
    {
        TX_data[0] = 0x10;
        TX_data[1] = 0x02;// Page Program CMD
        TX_data[2] = (addr + 128 * j) >> 16;
        TX_data[3] = (addr + 128 * j) >> 8;
        TX_data[4] = (addr + 128 * j);

        for(i = 0; i < 128; i++)
        {
            TX_data[5 + i] = pDataToWrite[j * 128 + i];
        }
        msleep(1);        // delay about 100us*****

        timeOutCount = 0;
        while ( ( drvISP_ReadStatus() & 0x01 ) == 0x01 )
        {
            timeOutCount++;
            if ( timeOutCount >= 100000 ) break; /* around 1 sec timeout */
        }

        drvISP_WriteEnable();
        i2c_write_update_msg2133( TX_data, 133);   //write 133 byte per cycle
        i2c_write_update_msg2133(&bWriteData1, 1);
    }
}

static void auto_drvISP_Program(unsigned char *fw, int size)
{
	unsigned short i = 0;
	unsigned short j = 0;
	unsigned short time = size/128;
	unsigned char TX_data[133];
	unsigned char bWriteData1 = 0x12;
	unsigned int addr;
	for(j = 0; j < time; j++){    //128*8 cycle
		addr = j * 128;
	        TX_data[0] = 0x10;
	        TX_data[1] = 0x02;// Page Program CMD
	        TX_data[2] = addr >> 16;
	        TX_data[3] = addr >> 8;
	        TX_data[4] = addr;
			
	        for(i = 0; i < 128; i++){
	            TX_data[5 + i] = fw[addr + i];
	        }

	        while((drvISP_ReadStatus() & 0x01) == 0x01)
	        {
	            ;    //wait until not in write operation
	        }

	        drvISP_WriteEnable();
	        i2c_write_update_msg2133( TX_data, 133);   //write 133 byte per cycle
	        i2c_write_update_msg2133(&bWriteData1, 1);
    }
}

static void drvISP_Verify ( unsigned short k, unsigned char* pDataToVerify )//new, check
{
    unsigned short i = 0, j = 0;
    unsigned char bWriteData[5] =
    {
        0x10, 0x03, 0, 0, 0
    };
    unsigned char RX_data[256];
    unsigned char bWriteData1 = 0x12;
    unsigned int addr = k * 1024;
    unsigned char index = 0;
    unsigned int timeOutCount;
    for ( j = 0; j < 8; j++ ) //128*8 cycle
    {
        bWriteData[2] = ( unsigned char ) ( ( addr + j * 128 ) >> 16 );
        bWriteData[3] = ( unsigned char ) ( ( addr + j * 128 ) >> 8 );
        bWriteData[4] = ( unsigned char ) ( addr + j * 128 );
        msleep(1);        // delay about 100us*****

        timeOutCount = 0;
        while ( ( drvISP_ReadStatus() & 0x01 ) == 0x01 )
        {
            timeOutCount++;
            if ( timeOutCount >= 100000 ) break; /* around 1 sec timeout */
        }

        i2c_write_update_msg2133 ( bWriteData, 5 ); //write read flash addr
        msleep(1);        // delay about 100us*****
        drvISP_Read ( 128, RX_data );
        i2c_write_update_msg2133 ( &bWriteData1, 1 ); //cmd end
        for ( i = 0; i < 128; i++ ) //log out if verify error
        {
            if ( ( RX_data[i] != 0 ) && index < 10 )
            {
                //TP_DEBUG("j=%d,RX_data[%d]=0x%x\n",j,i,RX_data[i]);
                index++;
            }
            if ( RX_data[i] != pDataToVerify[128 * j + i] )
            {
                printk( "k=%d,j=%d,i=%d===============Update Firmware Error================", k, j, i );
            }
        }
    }
}

static void _HalTscrHWReset ( void )//This function must implement by customer
{
#if 1
    gpio_direction_output ( MSG21XX_RESET_GPIO, 1 );
    gpio_set_value ( MSG21XX_RESET_GPIO, 1 );
    gpio_set_value ( MSG21XX_RESET_GPIO, 0 );
    mdelay ( 10 ); /* Note that the RST must be in LOW 10ms at least */
    gpio_set_value ( MSG21XX_RESET_GPIO, 1 );
    /* Enable the interrupt service thread/routine for INT after 50ms */
    mdelay ( 200/*50*/ );
#endif


}


static ssize_t firmware_update_c2 ( struct device *dev,
                                    struct device_attribute *attr, const char *buf, size_t size )//new, check
{
    unsigned char i;
    unsigned char dbbus_tx_data[4];
    unsigned char dbbus_rx_data[2] = {0};

    // set FRO to 50M
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x11;
    dbbus_tx_data[2] = 0xE2;
    i2c_write_msg2133 ( dbbus_tx_data, 3 );
    dbbus_rx_data[0] = 0;
    dbbus_rx_data[1] = 0;
    i2c_read_msg2133 ( &dbbus_rx_data[0], 2 );
    printk( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
    dbbus_tx_data[3] = dbbus_rx_data[0] & 0xF7;  //Clear Bit 3
    i2c_write_msg2133 ( dbbus_tx_data, 4 );

    // set MCU clock,SPI clock =FRO
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x22;
    dbbus_tx_data[3] = 0x00;
    i2c_write_msg2133 ( dbbus_tx_data, 4 );

    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x23;
    dbbus_tx_data[3] = 0x00;
    i2c_write_msg2133 ( dbbus_tx_data, 4 );

    // Enable slave's ISP ECO mode
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x08;
    dbbus_tx_data[2] = 0x0c;
    dbbus_tx_data[3] = 0x08;
    i2c_write_msg2133 ( dbbus_tx_data, 4 );

    // Enable SPI Pad
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x02;
    i2c_write_msg2133 ( dbbus_tx_data, 3 );
    i2c_read_msg2133 ( &dbbus_rx_data[0], 2 );
    printk( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
    dbbus_tx_data[3] = ( dbbus_rx_data[0] | 0x20 ); //Set Bit 5
    i2c_write_msg2133 ( dbbus_tx_data, 4 );

    // WP overwrite
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x0E;
    dbbus_tx_data[3] = 0x02;
    i2c_write_msg2133 ( dbbus_tx_data, 4 );

    // set pin high
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x10;
    dbbus_tx_data[3] = 0x08;
    i2c_write_msg2133 ( dbbus_tx_data, 4 );

    dbbusDWIICIICNotUseBus();
    dbbusDWIICNotStopMCU();
    dbbusDWIICExitSerialDebugMode();

    drvISP_EntryIspMode();
    drvISP_ChipErase();//this might need check
    _HalTscrHWReset();//this might need implement by customer
    mdelay ( 300 );

    // Program and Verify
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();

    // Disable the Watchdog
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x60;
    dbbus_tx_data[3] = 0x55;
    i2c_write_msg2133 ( dbbus_tx_data, 4 );
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x61;
    dbbus_tx_data[3] = 0xAA;
    i2c_write_msg2133 ( dbbus_tx_data, 4 );

    //Stop MCU
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x0F;
    dbbus_tx_data[2] = 0xE6;
    dbbus_tx_data[3] = 0x01;
    i2c_write_msg2133 ( dbbus_tx_data, 4 );

    // set FRO to 50M
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x11;
    dbbus_tx_data[2] = 0xE2;
    i2c_write_msg2133 ( dbbus_tx_data, 3 );
    dbbus_rx_data[0] = 0;
    dbbus_rx_data[1] = 0;
    i2c_read_msg2133 ( &dbbus_rx_data[0], 2 );
    printk ( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
    dbbus_tx_data[3] = dbbus_rx_data[0] & 0xF7;  //Clear Bit 3
    i2c_write_msg2133 ( dbbus_tx_data, 4 );

    // set MCU clock,SPI clock =FRO
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x22;
    dbbus_tx_data[3] = 0x00;
    i2c_write_msg2133 ( dbbus_tx_data, 4 );

    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x23;
    dbbus_tx_data[3] = 0x00;
    i2c_write_msg2133 ( dbbus_tx_data, 4 );

    // Enable slave's ISP ECO mode
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x08;
    dbbus_tx_data[2] = 0x0c;
    dbbus_tx_data[3] = 0x08;
    i2c_write_msg2133 ( dbbus_tx_data, 4 );

    // Enable SPI Pad
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x02;
    i2c_write_msg2133 ( dbbus_tx_data, 3 );
    i2c_read_msg2133 ( &dbbus_rx_data[0], 2 );
    printk ( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
    dbbus_tx_data[3] = ( dbbus_rx_data[0] | 0x20 ); //Set Bit 5
    i2c_write_msg2133 ( dbbus_tx_data, 4 );

    // WP overwrite
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x0E;
    dbbus_tx_data[3] = 0x02;
    i2c_write_msg2133 ( dbbus_tx_data, 4 );

    // set pin high
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x10;
    dbbus_tx_data[3] = 0x08;
    i2c_write_msg2133 ( dbbus_tx_data, 4 );

    dbbusDWIICIICNotUseBus();
    dbbusDWIICNotStopMCU();
    dbbusDWIICExitSerialDebugMode();

    ///////////////////////////////////////
    // Start to load firmware
    ///////////////////////////////////////
    drvISP_EntryIspMode();

    for ( i = 0; i < 94; i++ ) // total  94 KB : 1 byte per R/W
    {
        drvISP_Program ( i, temp[i] ); // program to slave's flash
        drvISP_Verify ( i, temp[i] ); //verify data
    }
    printk ( "update OK\n" );
    drvISP_ExitIspMode();
    FwDataCnt = 0;
    return size;
}

static void drvDB_WriteReg ( unsigned char bank, unsigned char addr, unsigned short data )//New. Check
{
    unsigned char tx_data[5] = {0x10, bank, addr, data & 0xFF, data >> 8};
    i2c_write_msg2133 ( tx_data, 5 );
}

static void drvDB_WriteReg8Bit ( unsigned char bank, unsigned char addr, unsigned char data )//New. Check
{
    unsigned char tx_data[4] = {0x10, bank, addr, data};
    i2c_write_msg2133 ( tx_data, 4 );
}

static unsigned short drvDB_ReadReg ( unsigned char bank, unsigned char addr )//New. Check
{
    unsigned char tx_data[3] = {0x10, bank, addr};
    unsigned char rx_data[2] = {0};

    i2c_write_msg2133 ( tx_data, 3 );
    i2c_read_msg2133 ( &rx_data[0], 2 );
    return ( rx_data[1] << 8 | rx_data[0] );
}

static unsigned int Reflect ( unsigned int ref, char ch )////New. Check
{
    unsigned int value = 0;
    unsigned int i = 0;

    for ( i = 1; i < ( ch + 1 ); i++ )
    {
        if ( ref & 1 )
        {
            value |= 1 << ( ch - i );
        }
        ref >>= 1;
    }
    return value;
}

static void Init_CRC32_Table ( unsigned int *crc32_table )//New. Check
{
    unsigned int magicnumber = 0x04c11db7;
    unsigned int i = 0, j;

    for ( i = 0; i <= 0xFF; i++ )
    {
        crc32_table[i] = Reflect ( i, 8 ) << 24;
        for ( j = 0; j < 8; j++ )
        {
            crc32_table[i] = ( crc32_table[i] << 1 ) ^ ( crc32_table[i] & ( 0x80000000L ) ? magicnumber : 0 );
        }
        crc32_table[i] = Reflect ( crc32_table[i], 32 );
    }
}

unsigned int Get_CRC ( unsigned int text, unsigned int prevCRC, unsigned int *crc32_table )//New. Check
{
    unsigned int ulCRC = prevCRC;
    {
        ulCRC = ( ulCRC >> 8 ) ^ crc32_table[ ( ulCRC & 0xFF ) ^ text];
    }
    return ulCRC ;
}


static int drvTP_erase_emem_c32 ( void )//New. Check
{
    /////////////////////////
    //Erase  all
    /////////////////////////
    
    //enter gpio mode
    drvDB_WriteReg ( 0x16, 0x1E, 0xBEAF );

    // before gpio mode, set the control pin as the orginal status
    drvDB_WriteReg ( 0x16, 0x08, 0x0000 );
    drvDB_WriteReg8Bit ( 0x16, 0x0E, 0x10 );
    mdelay ( 10 ); //MCR_CLBK_DEBUG_DELAY ( 10, MCU_LOOP_DELAY_COUNT_MS );

    // ptrim = 1, h'04[2]
    drvDB_WriteReg8Bit ( 0x16, 0x08, 0x04 );
    drvDB_WriteReg8Bit ( 0x16, 0x0E, 0x10 );
    mdelay ( 10 ); //MCR_CLBK_DEBUG_DELAY ( 10, MCU_LOOP_DELAY_COUNT_MS );

    // ptm = 6, h'04[12:14] = b'110
    drvDB_WriteReg8Bit ( 0x16, 0x09, 0x60 );
    drvDB_WriteReg8Bit ( 0x16, 0x0E, 0x10 );

    // pmasi = 1, h'04[6]
    drvDB_WriteReg8Bit ( 0x16, 0x08, 0x44 );
    // pce = 1, h'04[11]
    drvDB_WriteReg8Bit ( 0x16, 0x09, 0x68 );
    // perase = 1, h'04[7]
    drvDB_WriteReg8Bit ( 0x16, 0x08, 0xC4 );
    // pnvstr = 1, h'04[5]
    drvDB_WriteReg8Bit ( 0x16, 0x08, 0xE4 );
    // pwe = 1, h'04[9]
    drvDB_WriteReg8Bit ( 0x16, 0x09, 0x6A );
    // trigger gpio load
    drvDB_WriteReg8Bit ( 0x16, 0x0E, 0x10 );

    return ( 1 );
}

static ssize_t firmware_update_c32 ( struct device *dev, struct device_attribute *attr,
                                     const char *buf, size_t size,  EMEM_TYPE_t emem_type )//New, Check
{
    unsigned char  dbbus_tx_data[4];
    unsigned char  dbbus_rx_data[2] = {0};
    unsigned char  Fmr_Loader[1024];   // Buffer for slave's firmware

    unsigned int i, j;
    unsigned int crc_main, crc_main_tp;
    unsigned int crc_info, crc_info_tp;
    unsigned int crc_tab[256];
    unsigned short reg_data = 0;

    crc_main = 0xffffffff;
    crc_info = 0xffffffff;

#if 1
    /////////////////////////
    // Erase  all
    /////////////////////////
    drvTP_erase_emem_c32();
    mdelay ( 1000 ); //MCR_CLBK_DEBUG_DELAY ( 1000, MCU_LOOP_DELAY_COUNT_MS );

    //ResetSlave();
    _HalTscrHWReset();//This might implement by customer
    //drvDB_EnterDBBUS();
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    mdelay ( 300 );

    // Reset Watchdog
    drvDB_WriteReg8Bit ( 0x3C, 0x60, 0x55 );
    drvDB_WriteReg8Bit ( 0x3C, 0x61, 0xAA );

    /////////////////////////
    // Program
    /////////////////////////

    //polling 0x3CE4 is 0x1C70
    do
    {
        reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
    }
    while ( reg_data != 0x1C70 );


    drvDB_WriteReg ( 0x3C, 0xE4, 0xE38F );  // for all-blocks

    //polling 0x3CE4 is 0x2F43
    do
    {
        reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
    }
    while ( reg_data != 0x2F43 );


    //calculate CRC 32
    Init_CRC32_Table ( &crc_tab[0] );

    for ( i = 0; i < 33; i++ ) // total  33 KB : 2 byte per R/W
    {
        if ( i < 32 )   //emem_main
        {
            if ( i == 31 )
            {
                temp[i][1014] = 0x5A; //Fmr_Loader[1014]=0x5A;
                temp[i][1015] = 0xA5; //Fmr_Loader[1015]=0xA5;

                for ( j = 0; j < 1016; j++ )
                {
                    //crc_main=Get_CRC(Fmr_Loader[j],crc_main,&crc_tab[0]);
                    crc_main = Get_CRC ( temp[i][j], crc_main, &crc_tab[0] );
                }
            }
            else
            {
                for ( j = 0; j < 1024; j++ )
                {
                    //crc_main=Get_CRC(Fmr_Loader[j],crc_main,&crc_tab[0]);
                    crc_main = Get_CRC ( temp[i][j], crc_main, &crc_tab[0] );
                }
            }
        }
        else  // emem_info
        {
            for ( j = 0; j < 1024; j++ )
            {
                //crc_info=Get_CRC(Fmr_Loader[j],crc_info,&crc_tab[0]);
                crc_info = Get_CRC ( temp[i][j], crc_info, &crc_tab[0] );
            }
        }

        //drvDWIIC_MasterTransmit( DWIIC_MODE_DWIIC_ID, 1024, Fmr_Loader );
        msg2133_i2c_write( temp[i], 1024 );//////////////////////

        // polling 0x3CE4 is 0xD0BC
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
        }
        while ( reg_data != 0xD0BC );

        drvDB_WriteReg ( 0x3C, 0xE4, 0x2F43 );
    }

    //write file done
    drvDB_WriteReg ( 0x3C, 0xE4, 0x1380 );

    mdelay ( 10 ); //MCR_CLBK_DEBUG_DELAY ( 10, MCU_LOOP_DELAY_COUNT_MS );
    // polling 0x3CE4 is 0x9432
    do
    {
        reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
    }
    while ( reg_data != 0x9432 );

    crc_main = crc_main ^ 0xffffffff;
    crc_info = crc_info ^ 0xffffffff;

    // CRC Main from TP
    crc_main_tp = drvDB_ReadReg ( 0x3C, 0x80 );
    crc_main_tp = ( crc_main_tp << 16 ) | drvDB_ReadReg ( 0x3C, 0x82 );
 
    //CRC Info from TP
    crc_info_tp = drvDB_ReadReg ( 0x3C, 0xA0 );
    crc_info_tp = ( crc_info_tp << 16 ) | drvDB_ReadReg ( 0x3C, 0xA2 );

    printk ( "crc_main=0x%x, crc_info=0x%x, crc_main_tp=0x%x, crc_info_tp=0x%x\n",
               crc_main, crc_info, crc_main_tp, crc_info_tp );

    //drvDB_ExitDBBUS();
    if ( ( crc_main_tp != crc_main ) || ( crc_info_tp != crc_info ) )
    {
        printk ( "update FAILED\n" );
        FwDataCnt = 0;
        return ( 0 );
    }

    printk ( "update OK\n" );
    FwDataCnt = 0;
    return size;

#endif
}

static int drvTP_erase_emem_c33 ( EMEM_TYPE_t emem_type )//New, Check
{
    // stop mcu
    drvDB_WriteReg ( 0x0F, 0xE6, 0x0001 );

    //disable watch dog
    drvDB_WriteReg8Bit ( 0x3C, 0x60, 0x55 );
    drvDB_WriteReg8Bit ( 0x3C, 0x61, 0xAA );

    // set PROGRAM password
    drvDB_WriteReg8Bit ( 0x16, 0x1A, 0xBA );
    drvDB_WriteReg8Bit ( 0x16, 0x1B, 0xAB );

    //proto.MstarWriteReg(F1.loopDevice, 0x1618, 0x80);
    drvDB_WriteReg8Bit ( 0x16, 0x18, 0x80 );

    if ( emem_type == EMEM_ALL )
    {
        drvDB_WriteReg8Bit ( 0x16, 0x08, 0x10 ); //mark
    }

    drvDB_WriteReg8Bit ( 0x16, 0x18, 0x40 );
    mdelay ( 10 );

    drvDB_WriteReg8Bit ( 0x16, 0x18, 0x80 );

    // erase trigger
    if ( emem_type == EMEM_MAIN )
    {
        drvDB_WriteReg8Bit ( 0x16, 0x0E, 0x04 ); //erase main
    }
    else
    {
        drvDB_WriteReg8Bit ( 0x16, 0x0E, 0x08 ); //erase all block
    }

    return ( 1 );
}

static int drvTP_read_emem_dbbus_c33 ( EMEM_TYPE_t emem_type, u16 addr, size_t size, unsigned char *p, bool_t set_pce_high )//New, Check
{
    unsigned int i;

    // Set the starting address ( must before enabling burst mode and enter riu mode )
    drvDB_WriteReg ( 0x16, 0x00, addr );

    // Enable the burst mode ( must before enter riu mode )
    drvDB_WriteReg ( 0x16, 0x0C, drvDB_ReadReg ( 0x16, 0x0C ) | 0x0001 );

    // Set the RIU password
    drvDB_WriteReg ( 0x16, 0x1A, 0xABBA );

    // Enable the information block if pifren is HIGH
    if ( emem_type == EMEM_INFO )
    {
        // Clear the PCE
        drvDB_WriteReg ( 0x16, 0x18, drvDB_ReadReg ( 0x16, 0x18 ) | 0x0080 );
        mdelay ( 10 );

        // Set the PIFREN to be HIGH
        drvDB_WriteReg ( 0x16, 0x08, 0x0010 );
    }

    // Set the PCE to be HIGH
    drvDB_WriteReg ( 0x16, 0x18, drvDB_ReadReg ( 0x16, 0x18 ) | 0x0040 );
    mdelay ( 10 );

    // Wait pce becomes 1 ( read data ready )
    while ( ( drvDB_ReadReg ( 0x16, 0x10 ) & 0x0004 ) != 0x0004 );

    for ( i = 0; i < size; i += 4 )
    {
        // Fire the FASTREAD command
        drvDB_WriteReg ( 0x16, 0x0E, drvDB_ReadReg ( 0x16, 0x0E ) | 0x0001 );

        // Wait the operation is done
        while ( ( drvDB_ReadReg ( 0x16, 0x10 ) & 0x0001 ) != 0x0001 );

        p[i + 0] = drvDB_ReadReg ( 0x16, 0x04 ) & 0xFF;
        p[i + 1] = ( drvDB_ReadReg ( 0x16, 0x04 ) >> 8 ) & 0xFF;
        p[i + 2] = drvDB_ReadReg ( 0x16, 0x06 ) & 0xFF;
        p[i + 3] = ( drvDB_ReadReg ( 0x16, 0x06 ) >> 8 ) & 0xFF;
    }

    // Disable the burst mode
    drvDB_WriteReg ( 0x16, 0x0C, drvDB_ReadReg ( 0x16, 0x0C ) & ( ~0x0001 ) );

    // Clear the starting address
    drvDB_WriteReg ( 0x16, 0x00, 0x0000 );

    //Always return to main block
    if ( emem_type == EMEM_INFO )
    {
        // Clear the PCE before change block
        drvDB_WriteReg ( 0x16, 0x18, drvDB_ReadReg ( 0x16, 0x18 ) | 0x0080 );
        mdelay ( 10 );
        // Set the PIFREN to be LOW
        drvDB_WriteReg ( 0x16, 0x08, drvDB_ReadReg ( 0x16, 0x08 ) & ( ~0x0010 ) );

        drvDB_WriteReg ( 0x16, 0x18, drvDB_ReadReg ( 0x16, 0x18 ) | 0x0040 );
        while ( ( drvDB_ReadReg ( 0x16, 0x10 ) & 0x0004 ) != 0x0004 );
    }

    // Clear the RIU password
    drvDB_WriteReg ( 0x16, 0x1A, 0x0000 );

    if ( set_pce_high )
    {
        // Set the PCE to be HIGH before jumping back to e-flash codes
        drvDB_WriteReg ( 0x16, 0x18, drvDB_ReadReg ( 0x16, 0x18 ) | 0x0040 );
        while ( ( drvDB_ReadReg ( 0x16, 0x10 ) & 0x0004 ) != 0x0004 );
    }

    return ( 1 );
}

static int drvTP_read_info_dwiic_c33 ( void )//New, Check
{
    unsigned char dwiic_tx_data[5];

    //drvDB_EnterDBBUS();
     _HalTscrHWReset();
     mdelay ( 300 );
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    mdelay ( 300 );

    // Stop Watchdog
    drvDB_WriteReg8Bit ( 0x3C, 0x60, 0x55 );
    drvDB_WriteReg8Bit ( 0x3C, 0x61, 0xAA );

    drvDB_WriteReg ( 0x3C, 0xE4, 0xA4AB );

    // TP SW reset
    drvDB_WriteReg ( 0x1E, 0x02, 0x829F );

    mdelay ( 50 );

    dwiic_tx_data[0] = 0x72;
    dwiic_tx_data[1] = 0x80;
    dwiic_tx_data[2] = 0x00;
    dwiic_tx_data[3] = 0x04;
    dwiic_tx_data[4] = 0x00;
    msg2133_i2c_write ( dwiic_tx_data, 5 );

    mdelay ( 50 );

    // recive info data
    msg2133_i2c_read ( &g_dwiic_info_data[0], 1024 );

    return ( 1 );
}

static int drvTP_info_updata_C33 ( unsigned short start_index, unsigned char *data, unsigned short size )//New, check
{
    // size != 0, start_index+size !> 1024
    unsigned short i;
    
    for ( i = 0;i < size; i++ )
    {
        g_dwiic_info_data[start_index] = * ( data + i );
        start_index++;
    }
    
    return ( 1 );
}

static ssize_t firmware_update_c33 ( struct device *dev, struct device_attribute *attr,
                                     const char *buf, size_t size, EMEM_TYPE_t emem_type )//New, check
{
    unsigned char dbbus_tx_data[4];
    unsigned char  dbbus_rx_data[2] = {0};
    unsigned char  life_counter[2];
    unsigned int i, j;
    unsigned int crc_main, crc_main_tp;
    unsigned int crc_info, crc_info_tp;
    unsigned int crc_tab[256];

    int update_pass = 1;
    unsigned short reg_data = 0;

    crc_main = 0xffffffff;
    crc_info = 0xffffffff;

    drvTP_read_info_dwiic_c33();

    if ( g_dwiic_info_data[0] == 'M' && g_dwiic_info_data[1] == 'S' && g_dwiic_info_data[2] == 'T' && g_dwiic_info_data[3] == 'A' && g_dwiic_info_data[4] == 'R' && g_dwiic_info_data[5] == 'T' && g_dwiic_info_data[6] == 'P' && g_dwiic_info_data[7] == 'C' )
    {
        // updata FW Version
        drvTP_info_updata_C33 ( 8, temp[32][8], 4 );

        // updata life counter
        life_counter[0] = ( ( g_dwiic_info_data[12] << 8 + g_dwiic_info_data[13] + 1 ) >> 8 ) & 0xFF;
        life_counter[1] = ( g_dwiic_info_data[12] << 8 + g_dwiic_info_data[13] + 1 ) & 0xFF;
        drvTP_info_updata_C33 ( 10, life_counter, 2 );

        drvDB_WriteReg ( 0x3C, 0xE4, 0x78C5 );

        // TP SW reset
        drvDB_WriteReg ( 0x1E, 0x02, 0x829F );

        mdelay ( 50 );

        //polling 0x3CE4 is 0x2F43
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
        }
        while ( reg_data != 0x2F43 );

        // transmit lk info data
        msg2133_i2c_write ( g_dwiic_info_data, 1024 );

        //polling 0x3CE4 is 0xD0BC
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
        }
        while ( reg_data != 0xD0BC );

    }

    //erase main
    drvTP_erase_emem_c33 ( EMEM_MAIN );
    mdelay ( 1000 );

    //ResetSlave();
    _HalTscrHWReset();

    //drvDB_EnterDBBUS();
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    mdelay ( 300 );

    /////////////////////////
    // Program
    /////////////////////////

    //polling 0x3CE4 is 0x1C70
    if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
        }
        while ( reg_data != 0x1C70 );
    }

    switch ( emem_type )
    {
        case EMEM_ALL:
            drvDB_WriteReg ( 0x3C, 0xE4, 0xE38F );  // for all-blocks
            break;
        case EMEM_MAIN:
            drvDB_WriteReg ( 0x3C, 0xE4, 0x7731 );  // for main block
            break;
        case EMEM_INFO:
            drvDB_WriteReg ( 0x3C, 0xE4, 0x7731 );  // for info block


            drvDB_WriteReg8Bit ( 0x0F, 0xE6, 0x01 );

            drvDB_WriteReg8Bit ( 0x3C, 0xE4, 0xC5 ); //
            drvDB_WriteReg8Bit ( 0x3C, 0xE5, 0x78 ); //

            drvDB_WriteReg8Bit ( 0x1E, 0x04, 0x9F );
            drvDB_WriteReg8Bit ( 0x1E, 0x05, 0x82 );

            drvDB_WriteReg8Bit ( 0x0F, 0xE6, 0x00 );
            mdelay ( 100 );
            break;
    }

    // polling 0x3CE4 is 0x2F43
    do
    {
        reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
    }
    while ( reg_data != 0x2F43 );


    // calculate CRC 32
//    Iniift_CRC32_Table ( &crc_tab[0] );


    for ( i = 0; i < 33; i++ ) // total  33 KB : 2 byte per R/W
    {
         if( emem_type == EMEM_INFO ) i = 32;

        if ( i < 32 )   //emem_main
        {
            if ( i == 31 )
            {
                temp[i][1014] = 0x5A; //Fmr_Loader[1014]=0x5A;
                temp[i][1015] = 0xA5; //Fmr_Loader[1015]=0xA5;

                for ( j = 0; j < 1016; j++ )
                {
                    //crc_main=Get_CRC(Fmr_Loader[j],crc_main,&crc_tab[0]);
                    crc_main = Get_CRC ( temp[i][j], crc_main, &crc_tab[0] );
                }
            }
            else
            {
                for ( j = 0; j < 1024; j++ )
                {
                    //crc_main=Get_CRC(Fmr_Loader[j],crc_main,&crc_tab[0]);
                    crc_main = Get_CRC ( temp[i][j], crc_main, &crc_tab[0] );
                }
            }
        }
        else  //emem_info
        {
            for ( j = 0; j < 1024; j++ )
            {
                //crc_info=Get_CRC(Fmr_Loader[j],crc_info,&crc_tab[0]);
                crc_info = Get_CRC ( g_dwiic_info_data[j], crc_info, &crc_tab[0] );
            }
            if ( emem_type == EMEM_MAIN ) break;
        }

        //drvDWIIC_MasterTransmit( DWIIC_MODE_DWIIC_ID, 1024, Fmr_Loader );
        msg2133_i2c_write ( temp[i], 1024 );

        // polling 0x3CE4 is 0xD0BC
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
        }
        while ( reg_data != 0xD0BC );

        drvDB_WriteReg ( 0x3C, 0xE4, 0x2F43 );
    }

    if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        // write file done and check crc
        drvDB_WriteReg ( 0x3C, 0xE4, 0x1380 );
    }

    mdelay ( 10 ); //MCR_CLBK_DEBUG_DELAY ( 10, MCU_LOOP_DELAY_COUNT_MS );

    if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        // polling 0x3CE4 is 0x9432
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
        }
        while ( reg_data != 0x9432 );
    }

    crc_main = crc_main ^ 0xffffffff;
    crc_info = crc_info ^ 0xffffffff;

    if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        // CRC Main from TP
        crc_main_tp = drvDB_ReadReg ( 0x3C, 0x80 );
        crc_main_tp = ( crc_main_tp << 16 ) | drvDB_ReadReg ( 0x3C, 0x82 );

        // CRC Info from TP
        crc_info_tp = drvDB_ReadReg ( 0x3C, 0xA0 );
        crc_info_tp = ( crc_info_tp << 16 ) | drvDB_ReadReg ( 0x3C, 0xA2 );
    }

    printk ( "crc_main=0x%x, crc_info=0x%x, crc_main_tp=0x%x, crc_info_tp=0x%x\n",
               crc_main, crc_info, crc_main_tp, crc_info_tp );

    //drvDB_ExitDBBUS();

    update_pass = 1;

    _HalTscrHWReset();

    mdelay( 300 );	

/*
                  if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        if ( crc_main_tp != crc_main )
            update_pass = 0;

        if ( crc_info_tp != crc_info )
            update_pass = 0;
    }
*/

    if ( !update_pass )
    {
        printk ( "update FAILED\n" );
        FwDataCnt = 0;
//        return ( 0 );
           return size;
    }

    printk ( "update OK\n" );
    FwDataCnt = 0;
    
    return size;
}

static ssize_t firmware_update_show(struct device *dev,
                                    struct device_attribute *attr, char *buf)
{
	printk("tyd-tp: firmware_update_show\n");
	return sprintf(buf, "%03d%03d\n", fw_v.major, fw_v.minor);
}


#ifdef _FW_UPDATE_C3_
static ssize_t firmware_update_store ( struct device *dev,
                                       struct device_attribute *attr, const char *buf, size_t size )//New, check
{
	static int first = 0;
    unsigned char i;
    unsigned char dbbus_tx_data[4];
    unsigned char dbbus_rx_data[2] = {0};


    _HalTscrHWReset();

    mdelay ( 500 );

    // Erase TP Flash first
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    mdelay ( 300 );

    // Disable the Watchdog
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x60;
    dbbus_tx_data[3] = 0x55;
    i2c_write_msg2133 ( dbbus_tx_data, 4 );
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x61;
    dbbus_tx_data[3] = 0xAA;
    i2c_write_msg2133 ( dbbus_tx_data, 4 );

    // Stop MCU
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x0F;
    dbbus_tx_data[2] = 0xE6;
    dbbus_tx_data[3] = 0x01;
    i2c_write_msg2133 ( dbbus_tx_data, 4 );

    /////////////////////////
    // Difference between C2 and C3
    /////////////////////////

    //check id
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0xCC;
    i2c_write_msg2133 ( dbbus_tx_data, 3 );
    i2c_read_msg2133 ( &dbbus_rx_data[0], 2 );
    printk ( "dbbus_rx id[0]=0x%x \n", dbbus_rx_data[0] );

    if ( dbbus_rx_data[0] == 2 )
    {
        // check version
        dbbus_tx_data[0] = 0x10;
        dbbus_tx_data[1] = 0x3C;
        dbbus_tx_data[2] = 0xEA;
        i2c_write_msg2133 ( /*FW_ADDR_MSG21XX,*/ dbbus_tx_data, 3 );
        i2c_read_msg2133 ( /*FW_ADDR_MSG21XX,*/ &dbbus_rx_data[0], 2 );
        printk ( "dbbus_rx version[0]=0x%x  \n ", dbbus_rx_data[0] );

        if ( dbbus_rx_data[0] == 3 )
        {
		printk("check go firmware_update_c33\n");
		first =1;
            return firmware_update_c33 ( dev, attr, buf, size, EMEM_MAIN );
        }
        else
        {
		printk("check go firmware_update_c32\n");
            return firmware_update_c32 ( dev, attr, buf, size, EMEM_ALL );
        }
    }
    else
    {
		printk("check go firmware_update_c2\n");
	
        return firmware_update_c2 ( dev, attr, buf, size );
    }

}

#else

static ssize_t firmware_update_store ( struct device *dev,
                                       struct device_attribute *attr, const char *buf, size_t size )//New, check
{
    unsigned char i;
    unsigned char dbbus_tx_data[4];
    unsigned char dbbus_rx_data[2] = {0};


    _HalTscrHWReset();

    // 1. Erase TP Flash first
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    mdelay ( 300 );


    // Disable the Watchdog
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x60;
    dbbus_tx_data[3] = 0x55;
    i2c_write_msg2133 ( dbbus_tx_data, 4 );
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x61;
    dbbus_tx_data[3] = 0xAA;
    i2c_write_msg2133 ( dbbus_tx_data, 4 );

    // Stop MCU
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x0F;
    dbbus_tx_data[2] = 0xE6;
    dbbus_tx_data[3] = 0x01;
    i2c_write_msg2133 ( dbbus_tx_data, 4 );


    // set FRO to 50M
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x11;
    dbbus_tx_data[2] = 0xE2;
    i2c_write_msg2133 ( dbbus_tx_data, 3 );
    dbbus_rx_data[0] = 0;
    dbbus_rx_data[1] = 0;
    i2c_read_msg2133 ( &dbbus_rx_data[0], 2 );
    prink ( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
    dbbus_tx_data[3] = dbbus_rx_data[0] & 0xF7;  //Clear Bit 3
    i2c_write_msg2133 ( dbbus_tx_data, 4 );


    // set MCU clock,SPI clock =FRO
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x22;
    dbbus_tx_data[3] = 0x00;
    i2c_write_msg2133 ( dbbus_tx_data, 4 );

    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x23;
    dbbus_tx_data[3] = 0x00;
    i2c_write_msg2133 ( dbbus_tx_data, 4 );


    // Enable slave's ISP ECO mode
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x08;
    dbbus_tx_data[2] = 0x0c;
    dbbus_tx_data[3] = 0x08;
    i2c_write_msg2133 ( dbbus_tx_data, 4 );

    // Enable SPI Pad
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x02;
    i2c_write_msg2133 ( dbbus_tx_data, 3 );
    i2c_read_msg2133 ( &dbbus_rx_data[0], 2 );
    printk ( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
    dbbus_tx_data[3] = ( dbbus_rx_data[0] | 0x20 ); //Set Bit 5
    i2c_write_msg2133 ( dbbus_tx_data, 4 );


    // WP overwrite
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x0E;
    dbbus_tx_data[3] = 0x02;
    i2c_write_msg2133 ( dbbus_tx_data, 4 );


    // set pin high
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x10;
    dbbus_tx_data[3] = 0x08;
    i2c_write_msg2133 ( dbbus_tx_data, 4 );

    dbbusDWIICIICNotUseBus();
    dbbusDWIICNotStopMCU();
    dbbusDWIICExitSerialDebugMode();


    drvISP_EntryIspMode();
    drvISP_ChipErase();
    _HalTscrHWReset();
    mdelay ( 300 );

    // 2.Program and Verify
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();


    // Disable the Watchdog
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x60;
    dbbus_tx_data[3] = 0x55;
    i2c_write_msg2133 ( dbbus_tx_data, 4 );
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x61;
    dbbus_tx_data[3] = 0xAA;
    i2c_write_msg2133 ( dbbus_tx_data, 4 );

    // Stop MCU
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x0F;
    dbbus_tx_data[2] = 0xE6;
    dbbus_tx_data[3] = 0x01;
    i2c_write_msg2133 ( dbbus_tx_data, 4 );


    // set FRO to 50M
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x11;
    dbbus_tx_data[2] = 0xE2;
    i2c_write_msg2133 ( dbbus_tx_data, 3 );
    dbbus_rx_data[0] = 0;
    dbbus_rx_data[1] = 0;
    i2c_read_msg2133 ( &dbbus_rx_data[0], 2 );
    printk ( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
    dbbus_tx_data[3] = dbbus_rx_data[0] & 0xF7;  //Clear Bit 3
    i2c_write_msg2133 ( dbbus_tx_data, 4 );


    // set MCU clock,SPI clock =FRO
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x22;
    dbbus_tx_data[3] = 0x00;
    i2c_write_msg2133 ( dbbus_tx_data, 4 );

    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x23;
    dbbus_tx_data[3] = 0x00;
    i2c_write_msg2133 ( dbbus_tx_data, 4 );

    // Enable slave's ISP ECO mode
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x08;
    dbbus_tx_data[2] = 0x0c;
    dbbus_tx_data[3] = 0x08;
    i2c_write_msg2133 ( dbbus_tx_data, 4 );

    // Enable SPI Pad
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x02;
    i2c_write_msg2133 ( dbbus_tx_data, 3 );
    i2c_read_msg2133 ( &dbbus_rx_data[0], 2 );
    printk ( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
    dbbus_tx_data[3] = ( dbbus_rx_data[0] | 0x20 ); //Set Bit 5
    i2c_write_msg2133 ( dbbus_tx_data, 4 );


    // WP overwrite
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x0E;
    dbbus_tx_data[3] = 0x02;
    i2c_write_msg2133 ( dbbus_tx_data, 4 );


    // set pin high
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x10;
    dbbus_tx_data[3] = 0x08;
    i2c_write_msg2133 ( dbbus_tx_data, 4 );

    dbbusDWIICIICNotUseBus();
    dbbusDWIICNotStopMCU();
    dbbusDWIICExitSerialDebugMode();

    ///////////////////////////////////////
    // Start to load firmware
    ///////////////////////////////////////
    drvISP_EntryIspMode();

    for ( i = 0; i < 94; i++ ) // total  94 KB : 1 byte per R/W
    {
        drvISP_Program ( i, temp[i] ); // program to slave's flash
        drvISP_Verify ( i, temp[i] ); //verify data
    }
    printk ( "update OK\n" );
    drvISP_ExitIspMode();
    FwDataCnt = 0;
    
    return size;
}
#endif

static ssize_t firmware_version_show(struct device *dev,
                                     struct device_attribute *attr, char *buf)//Check
{
	printk("tyd-tp: firmware_version_show\n");
//	MSG2133_DBG("*** firmware_version_show fw_version = %03d.%03d.%02d***\n", fw_v.major, fw_v.minor,fw_v.VenderID);
	return sprintf(buf, "%03d%03d%02d\n", fw_v.major, fw_v.minor,fw_v.VenderID);
}

static ssize_t firmware_version_store(struct device *dev,
                                      struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned char dbbus_tx_data[3];
	unsigned char dbbus_rx_data[13] ;
	int time = 0;
	
	printk("%s\n", __func__);

	while(time < 10){
		dbbus_tx_data[0] = 0x53;
		dbbus_tx_data[1] = 0x00;
		dbbus_tx_data[2] = 0x74;
		msg2133_i2c_write(&dbbus_tx_data[0], 3);
		mdelay(50);
		msg2133_i2c_read(&dbbus_rx_data[0], 13);
		fw_v.major = (dbbus_rx_data[1] << 8) + dbbus_rx_data[0];
		fw_v.minor = (dbbus_rx_data[3] << 8) + dbbus_rx_data[2];
		fw_v.VenderID = dbbus_rx_data[4];
		
		time++;
	printk("%s, FW majoy = 0x%x, minor = 0x%x, Vender ID = 0x %x\n", __func__,
		fw_v.major, fw_v.minor, fw_v.VenderID);

		if((fw_v.major & 0xff52/*0xff00*/) == 0)
			break;
		msleep(50);
	}
	
	return 0;
}


static ssize_t firmware_data_show(struct device *dev,
                                  struct device_attribute *attr, char *buf)//Check
{
	printk("tyd-tp: firmware_data_show\n");
    	return FwDataCnt;
}

static ssize_t firmware_data_store(struct device *dev,
                                   struct device_attribute *attr, const char *buf, size_t size)//Check
{
    	int i;
    	MSG2133_DBG("***FwDataCnt = %d ***\n", FwDataCnt);
	printk("tyd-tp: firmware_data_store\n");
    	for(i = 0; i < 1024; i++){
        	memcpy(temp[FwDataCnt], buf, 1024);
    	}

    	FwDataCnt++;
    	return size;
}

static ssize_t firmware_clear_show(struct device *dev,
                                   struct device_attribute *attr, char *buf)//Check
{
    unsigned short k = 0, i = 0, j = 0;
    unsigned char bWriteData[5] =
    {
        0x10, 0x03, 0, 0, 0
    };
    unsigned char RX_data[256];
    unsigned char bWriteData1 = 0x12;
    unsigned int addr = 0;
    MSG2133_DBG("\n");
	printk("tyd-tp: firmware_clear_show\n");
    for(k = 0; k < 94; i++)    // total  94 KB : 1 byte per R/W
    {
        addr = k * 1024;

        for(j = 0; j < 8; j++)    //128*8 cycle
        {
            bWriteData[2] = (unsigned char)((addr + j * 128) >> 16);
            bWriteData[3] = (unsigned char)((addr + j * 128) >> 8);
            bWriteData[4] = (unsigned char)(addr + j * 128);

            while((drvISP_ReadStatus() & 0x01) == 0x01)
            {
                ;    //wait until not in write operation
            }

            i2c_write_update_msg2133(bWriteData, 5);     //write read flash addr
            drvISP_Read(128, RX_data);
            i2c_write_update_msg2133(&bWriteData1, 1);    //cmd end

            for(i = 0; i < 128; i++)    //log out if verify error{
                if(RX_data[i] != 0xFF){
                    MSG2133_DBG("k=%d,j=%d,i=%d===============erase not clean================", k, j, i);
                }
            }
     }
    MSG2133_DBG("read finish\n");
    return sprintf(buf, "%03d.%03d\n", fw_v.major, fw_v.minor);
}

static ssize_t firmware_clear_store(struct device *dev,
                                    struct device_attribute *attr, const char *buf, size_t size)
{
    unsigned char dbbus_tx_data[4];
    unsigned char dbbus_rx_data[2] = {0};
    //msctpc_LoopDelay ( 100 );        // delay about 100ms*****
    // Enable slave's ISP ECO mode
    
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    MSG2133_DBG("\n");
	printk("tyd-tp: firmware_clear_store\n");
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x08;
    dbbus_tx_data[2] = 0x0c;
    dbbus_tx_data[3] = 0x08;
    // Disable the Watchdog
    i2c_write_msg2133(dbbus_tx_data, 4);
    //Get_Chip_Version();
    //FwVersion  = 2;
    //if (FwVersion  == 2)
    {
		dbbus_tx_data[0] = 0x10;
		dbbus_tx_data[1] = 0x11;
		dbbus_tx_data[2] = 0xE2;
		dbbus_tx_data[3] = 0x00;
		i2c_write_msg2133(dbbus_tx_data, 4);
	}
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x3C;
	dbbus_tx_data[2] = 0x60;
	dbbus_tx_data[3] = 0x55;
	i2c_write_msg2133(dbbus_tx_data, 4);
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x3C;
	dbbus_tx_data[2] = 0x61;
	dbbus_tx_data[3] = 0xAA;
	i2c_write_msg2133(dbbus_tx_data, 4);
	//Stop MCU
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x0F;
	dbbus_tx_data[2] = 0xE6;
	dbbus_tx_data[3] = 0x01;
	i2c_write_msg2133(dbbus_tx_data, 4);
	//Enable SPI Pad
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x1E;
	dbbus_tx_data[2] = 0x02;
	i2c_write_msg2133(dbbus_tx_data, 3);
	i2c_read_msg2133(&dbbus_rx_data[0], 2);
	MSG2133_DBG("dbbus_rx_data[0]=0x%x", dbbus_rx_data[0]);
	dbbus_tx_data[3] = (dbbus_rx_data[0] | 0x20);  //Set Bit 5
	i2c_write_msg2133(dbbus_tx_data, 4);
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x1E;
	dbbus_tx_data[2] = 0x25;
	i2c_write_msg2133(dbbus_tx_data, 3);
	dbbus_rx_data[0] = 0;
	dbbus_rx_data[1] = 0;
	i2c_read_msg2133(&dbbus_rx_data[0], 2);
	MSG2133_DBG("dbbus_rx_data[0]=0x%x", dbbus_rx_data[0]);
	dbbus_tx_data[3] = dbbus_rx_data[0] & 0xFC;  //Clear Bit 1,0
	i2c_write_msg2133(dbbus_tx_data, 4);
	//WP overwrite
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x1E;
	dbbus_tx_data[2] = 0x0E;
	dbbus_tx_data[3] = 0x02;
	i2c_write_msg2133(dbbus_tx_data, 4);
	//set pin high
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x1E;
	dbbus_tx_data[2] = 0x10;
	dbbus_tx_data[3] = 0x08;
	i2c_write_msg2133(dbbus_tx_data, 4);
	dbbusDWIICIICNotUseBus();
	dbbusDWIICNotStopMCU();
	dbbusDWIICExitSerialDebugMode();
	///////////////////////////////////////
	// Start to load firmware
	///////////////////////////////////////
	drvISP_EntryIspMode();
	MSG2133_DBG("chip erase+\n");
	
	if(tp_type == TP_HUANGZE){
		drvISP_SectorErase(0x000000);
		drvISP_SectorErase(0x001000);
		drvISP_SectorErase(0x002000);
		drvISP_SectorErase(0x003000);
		drvISP_SectorErase(0x004000);
		drvISP_SectorErase(0x005000);
		drvISP_SectorErase(0x006000);
		drvISP_SectorErase(0x007000);
		drvISP_SectorErase(0x008000);
		drvISP_SectorErase(0x009000);
		drvISP_SectorErase(0x00a000);
		drvISP_SectorErase(0x00b000);
		drvISP_SectorErase(0x00c000);
		drvISP_SectorErase(0x00d000);
		drvISP_SectorErase(0x00e000);
		drvISP_SectorErase(0x00f000);

	}else{
		drvISP_BlockErase(0x00000);
	}  


	MSG2133_DBG("chip erase-\n");
	drvISP_ExitIspMode();
	return size;
}

static DEVICE_ATTR(version, 0644/*0777*/, firmware_version_show, firmware_version_store);
static DEVICE_ATTR(update, 0644/*0777*/, firmware_update_show, firmware_update_store);
static DEVICE_ATTR(data, 0644/*0777*/, firmware_data_show, firmware_data_store);
static DEVICE_ATTR(clear, 0644/*0777*/, firmware_clear_show, firmware_clear_store);

void msg2133_init_fw_class()
{
	firmware_class = class_create(THIS_MODULE,"mtk-tpd" );//client->name

	if(IS_ERR(firmware_class))
		pr_err("Failed to create class(firmware)!\n");

	firmware_cmd_dev = device_create(firmware_class,
	                                     NULL, 0, NULL, "device");//device

	if(IS_ERR(firmware_cmd_dev))
		pr_err("Failed to create device(firmware_cmd_dev)!\n");
		
	// version /sys/class/mtk-tpd/device/version
//	if(device_create_file(firmware_cmd_dev, &dev_attr_version) < 0)
//		pr_err("Failed to create device file(%s)!\n", dev_attr_version.attr.name);

	// update /sys/class/mtk-tpd/device/update
	if(device_create_file(firmware_cmd_dev, &dev_attr_update) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_update.attr.name);

	// data /sys/class/mtk-tpd/device/data
	if(device_create_file(firmware_cmd_dev, &dev_attr_data) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_data.attr.name);

	// clear /sys/class/mtk-tpd/device/clear
	if(device_create_file(firmware_cmd_dev, &dev_attr_clear) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_clear.attr.name);
}

static ssize_t msg2133_fw_update(unsigned char *fw, unsigned int size)
{
	unsigned char i;
	unsigned char dbbus_tx_data[4];
	unsigned char dbbus_rx_data[2] = {0};

	printk("%s, start update fireware\n", __func__);

	//¹ر?ж?
	disable_irq_nosync(this_client->irq);

//	msg2133_reset();
	_HalTscrHWReset();
	//msctpc_LoopDelay ( 100 );        // delay about 100ms*****
	// Enable slave's ISP ECO mode
	dbbusDWIICEnterSerialDebugMode();
	dbbusDWIICStopMCU();
	dbbusDWIICIICUseBus();
	dbbusDWIICIICReshape();
	//pr_ch("dbbusDWIICIICReshape\n");
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x08;
	dbbus_tx_data[2] = 0x0c;
	dbbus_tx_data[3] = 0x08;
	// Disable the Watchdog
	i2c_write_msg2133(dbbus_tx_data, 4);
	//Get_Chip_Version();
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x11;
	dbbus_tx_data[2] = 0xE2;
	dbbus_tx_data[3] = 0x00;
	i2c_write_msg2133(dbbus_tx_data, 4);
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x3C;
	dbbus_tx_data[2] = 0x60;
	dbbus_tx_data[3] = 0x55;
	i2c_write_msg2133(dbbus_tx_data, 4);
	//pr_ch("update\n");
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x3C;
	dbbus_tx_data[2] = 0x61;
	dbbus_tx_data[3] = 0xAA;
	i2c_write_msg2133(dbbus_tx_data, 4);
	//Stop MCU
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x0F;
	dbbus_tx_data[2] = 0xE6;
	dbbus_tx_data[3] = 0x01;
	i2c_write_msg2133(dbbus_tx_data, 4);
	//Enable SPI Pad
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x1E;
	dbbus_tx_data[2] = 0x02;
	i2c_write_msg2133(dbbus_tx_data, 3);
	i2c_read_msg2133(&dbbus_rx_data[0], 2);
	//pr_tp("dbbus_rx_data[0]=0x%x", dbbus_rx_data[0]);
	dbbus_tx_data[3] = (dbbus_rx_data[0] | 0x20);  //Set Bit 5
	i2c_write_msg2133(dbbus_tx_data, 4);
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x1E;
	dbbus_tx_data[2] = 0x25;
	i2c_write_msg2133(dbbus_tx_data, 3);
	dbbus_rx_data[0] = 0;
	dbbus_rx_data[1] = 0;
	i2c_read_msg2133(&dbbus_rx_data[0], 2);
	//pr_tp("dbbus_rx_data[0]=0x%x", dbbus_rx_data[0]);
	dbbus_tx_data[3] = dbbus_rx_data[0] & 0xFC;  //Clear Bit 1,0
	i2c_write_msg2133(dbbus_tx_data, 4);

	//WP overwrite
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x1E;
	dbbus_tx_data[2] = 0x0E;
	dbbus_tx_data[3] = 0x02;
	i2c_write_msg2133(dbbus_tx_data, 4);
	//set pin high
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x1E;
	dbbus_tx_data[2] = 0x10;
	dbbus_tx_data[3] = 0x08;
	i2c_write_msg2133(dbbus_tx_data, 4);
	dbbusDWIICIICNotUseBus();
	dbbusDWIICNotStopMCU();
	dbbusDWIICExitSerialDebugMode();
	///////////////////////////////////////
	// Start to load firmware
	///////////////////////////////////////
	drvISP_EntryIspMode();
	MSG2133_DBG("entryisp\n");
	if(tp_type == TP_HUANGZE){
		drvISP_SectorErase(0x000000);
		drvISP_SectorErase(0x001000);
		drvISP_SectorErase(0x002000);
		drvISP_SectorErase(0x003000);
		drvISP_SectorErase(0x004000);
		drvISP_SectorErase(0x005000);
		drvISP_SectorErase(0x006000);
		drvISP_SectorErase(0x007000);
		drvISP_SectorErase(0x008000);
		drvISP_SectorErase(0x009000);
		drvISP_SectorErase(0x00a000);
		drvISP_SectorErase(0x00b000);
		drvISP_SectorErase(0x00c000);
		drvISP_SectorErase(0x00d000);
		drvISP_SectorErase(0x00e000);
		drvISP_SectorErase(0x00f000);

	}else{
		drvISP_BlockErase(0x00000);
	}  


	//msleep(1000);
	MSG2133_DBG("FwVersion=2");

	MSG2133_DBG("drvISP_Program\n");
	auto_drvISP_Program(fw, size);


	//MSG2133_DBG("update OK\n");
	drvISP_ExitIspMode();
	_HalTscrHWReset();//msg2133_reset();
	printk("%s, update OK\n", __func__);
	//´�?ж?
	enable_irq(this_client->irq);
	return size;
}

static int msg2133_get_version(struct fw_version *fw)
{
	unsigned char dbbus_tx_data[3];
	unsigned char dbbus_rx_data[13] ;
	
	printk("%s\n", __func__);
/*	
	dbbusDWIICEnterSerialDebugMode();
	dbbusDWIICStopMCU();
	dbbusDWIICIICUseBus();
	dbbusDWIICIICReshape();
    */
	MSG2133_DBG("\n");
	dbbus_tx_data[0] = 0x53;
	dbbus_tx_data[1] = 0x00;
	dbbus_tx_data[2] = 0x74;
	msg2133_i2c_write(&dbbus_tx_data[0], 3);
	mdelay(50);
	msg2133_i2c_read(&dbbus_rx_data[0], 5);
	fw->major = (dbbus_rx_data[1] << 8) + dbbus_rx_data[0];
	fw->minor = (dbbus_rx_data[3] << 8) + dbbus_rx_data[2];
	fw->VenderID= dbbus_rx_data[4];
//	printk("%s, major = 0x%x, minor = 0x%x\n,VenderID = 0x%x\n,0x%x\n,0x%x\n,0x%x\n,0x%x\n,0x%x\n,0x%x\n,0x%x\n,0x%x\n", __func__, 
//		   fw->major, fw->minor,fw->VenderID,dbbus_rx_data[5],dbbus_rx_data[6],dbbus_rx_data[7],dbbus_rx_data[8],dbbus_rx_data[9],
//		   dbbus_rx_data[10] dbbus_rx_data[11],dbbus_rx_data[12]);
}


#ifdef MSG2133_AUTO_UPDATE
static int msg2133_auto_update_fw()
{
	int ret = 0, time=0;
	const struct firmware *fw;
	unsigned char *fw_buf;
	struct platform_device *pdev;
	struct fw_version fw_new, fw_old;
	
	printk("%s\n", __func__);
	
	pdev = platform_device_register_simple("msg2133_ts", 0, NULL, 0);
	if (IS_ERR(pdev)) {
		printk("%s: Failed to register firmware\n", __func__);
		return -1;
	}
	printk("%s, platform_device_register_simple\n", __func__);

	if(tp_type == TP_HUANGZE)
		ret = request_firmware(&fw, "huangze_fw.bin", &pdev->dev);
	else if(tp_type == TP_JUNDA)
		ret = request_firmware(&fw, "junda_fw.bin", &pdev->dev);
	else if(tp_type == TP_MUDOND)
		ret = request_firmware(&fw, "mudong_fw.bin", &pdev->dev);
	else
		return -1;
	
	if (ret) {
		printk("%s: request_firmware error\n",__func__);
		platform_device_unregister(pdev);
        	return -1;
    	}
	
	printk("%s, request_firmware\n", __func__);

	platform_device_unregister(pdev);
    	printk("%s:fw size=%d\n", __func__,fw->size);
	
    	fw_buf = kzalloc(fw->size, GFP_KERNEL | GFP_DMA);
    	memcpy(fw_buf, fw->data, fw->size);

	while(time < 10){
		msg2133_get_version(&fw_old);
		time++;
		if((fw_old.major & 0xff00) == 0)
			break;
		msleep(50);
	}
	printk("%s: time = %d, fw_old.major = 0x%x, fw_old.minor = 0x%x\n", __func__, time, fw_old.major, fw_old.minor);

	fw_new.major = (fw_buf[0x3076] << 8) + fw_buf[0x3077];
	fw_new.minor = (fw_buf[0x3074] << 8) + fw_buf[0x3075];
	//fw_new.minor = (fw_buf[0x3074] << 8) + fw_buf[0x3075];
	printk("%s: fw_new.major = 0x%x, fw_new.minor = 0x%x\n", __func__, fw_new.major, fw_new.minor);

	if((fw_old.major > fw_new.major) || ((fw_old.major == fw_new.major) && (fw_old.minor >= fw_new.minor)))
		return 0;

	printk("%s: update start\n", __func__);
	
   	msg2133_fw_update(fw_buf, fw->size);

	printk("%s: update finish\n", __func__);
	release_firmware(fw);
	kfree(fw_buf);

	return 0;    

}
#endif

#ifdef MSG2133_DRIVER_UPDATE


static int firmware_driver_update_c33 ( EMEM_TYPE_t emem_type )//New, check
{
    unsigned char dbbus_tx_data[4];
    unsigned char  dbbus_rx_data[2] = {0};
    unsigned char  life_counter[2];
    unsigned int i, j;
    unsigned int crc_main, crc_main_tp;
    unsigned int crc_info, crc_info_tp;
    unsigned int crc_tab[256];

    int update_pass = 1;
    unsigned short reg_data = 0;

    crc_main = 0xffffffff;
    crc_info = 0xffffffff;

   printk("__ %s __  (%d)\n", __func__, __LINE__);


	if (fw_version_now < 0x104){
	    for(i= 0; i<32;i++)
	    {
		for (j=0; j < 1024 ; j++)
				temp[i][j] = FW_PF[i*1024+j];

	    }
	}else if (fw_version_now < 0x109){
	    for(i= 0; i<32;i++)
	    {
		for (j=0; j < 1024 ; j++)
				temp[i][j] = FW_GF[i*1024+j];

	    }
	}else{

	    if (touch_board_id == NABIJR_BOARD_VER_EVT){
		    for(i= 0; i<32;i++)
		    {
			for (j=0; j < 1024 ; j++)
					temp[i][j] = FW_PF[i*1024+j];

		    }
	    }else{
		    for(i= 0; i<32;i++)
		    {
			for (j=0; j < 1024 ; j++)
					temp[i][j] = FW_GF[i*1024+j];

		    }
	    }
	}

    drvTP_read_info_dwiic_c33();

    if ( g_dwiic_info_data[0] == 'M' && g_dwiic_info_data[1] == 'S' && g_dwiic_info_data[2] == 'T' && g_dwiic_info_data[3] == 'A' && g_dwiic_info_data[4] == 'R' && g_dwiic_info_data[5] == 'T' && g_dwiic_info_data[6] == 'P' && g_dwiic_info_data[7] == 'C' )
    {
        // updata FW Version
        drvTP_info_updata_C33 ( 8, temp[32][8], 4 );

        // updata life counter
        life_counter[0] = ( ( g_dwiic_info_data[12] << 8 + g_dwiic_info_data[13] + 1 ) >> 8 ) & 0xFF;
        life_counter[1] = ( g_dwiic_info_data[12] << 8 + g_dwiic_info_data[13] + 1 ) & 0xFF;
        drvTP_info_updata_C33 ( 10, life_counter, 2 );

        drvDB_WriteReg ( 0x3C, 0xE4, 0x78C5 );

        // TP SW reset
        drvDB_WriteReg ( 0x1E, 0x02, 0x829F );

        mdelay ( 50 );

        //polling 0x3CE4 is 0x2F43
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
        }
        while ( reg_data != 0x2F43 );

        // transmit lk info data
        msg2133_i2c_write ( g_dwiic_info_data, 1024 );

        //polling 0x3CE4 is 0xD0BC
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
        }
        while ( reg_data != 0xD0BC );

    }

    //erase main
    drvTP_erase_emem_c33 ( EMEM_MAIN );
    mdelay ( 1000 );

    //ResetSlave();
    _HalTscrHWReset();

    //drvDB_EnterDBBUS();
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    mdelay ( 300 );

    /////////////////////////
    // Program
    /////////////////////////

    //polling 0x3CE4 is 0x1C70
    if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
        }
        while ( reg_data != 0x1C70 );
    }

    switch ( emem_type )
    {
        case EMEM_ALL:
            drvDB_WriteReg ( 0x3C, 0xE4, 0xE38F );  // for all-blocks
            break;
        case EMEM_MAIN:
            drvDB_WriteReg ( 0x3C, 0xE4, 0x7731 );  // for main block
            break;
        case EMEM_INFO:
            drvDB_WriteReg ( 0x3C, 0xE4, 0x7731 );  // for info block


            drvDB_WriteReg8Bit ( 0x0F, 0xE6, 0x01 );

            drvDB_WriteReg8Bit ( 0x3C, 0xE4, 0xC5 ); //
            drvDB_WriteReg8Bit ( 0x3C, 0xE5, 0x78 ); //

            drvDB_WriteReg8Bit ( 0x1E, 0x04, 0x9F );
            drvDB_WriteReg8Bit ( 0x1E, 0x05, 0x82 );

            drvDB_WriteReg8Bit ( 0x0F, 0xE6, 0x00 );
            mdelay ( 100 );
            break;
    }

    // polling 0x3CE4 is 0x2F43
    do
    {
        reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
    }
    while ( reg_data != 0x2F43 );


    // calculate CRC 32
//    Iniift_CRC32_Table ( &crc_tab[0] );


    for ( i = 0; i < 33; i++ ) // total  33 KB : 2 byte per R/W
    {
         if( emem_type == EMEM_INFO ) i = 32;

        if ( i < 32 )   //emem_main
        {
            if ( i == 31 )
            {
                temp[i][1014] = 0x5A; //Fmr_Loader[1014]=0x5A;
                temp[i][1015] = 0xA5; //Fmr_Loader[1015]=0xA5;

                for ( j = 0; j < 1016; j++ )
                {
                    //crc_main=Get_CRC(Fmr_Loader[j],crc_main,&crc_tab[0]);
                    crc_main = Get_CRC ( temp[i][j], crc_main, &crc_tab[0] );
                }
            }
            else
            {
                for ( j = 0; j < 1024; j++ )
                {
                    //crc_main=Get_CRC(Fmr_Loader[j],crc_main,&crc_tab[0]);
                    crc_main = Get_CRC ( temp[i][j], crc_main, &crc_tab[0] );
                }
            }
        }
        else  //emem_info
        {
            for ( j = 0; j < 1024; j++ )
            {
                //crc_info=Get_CRC(Fmr_Loader[j],crc_info,&crc_tab[0]);
                crc_info = Get_CRC ( g_dwiic_info_data[j], crc_info, &crc_tab[0] );
            }
            if ( emem_type == EMEM_MAIN ) break;
        }

        //drvDWIIC_MasterTransmit( DWIIC_MODE_DWIIC_ID, 1024, Fmr_Loader );
        msg2133_i2c_write ( temp[i], 1024 );

        // polling 0x3CE4 is 0xD0BC
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
        }
        while ( reg_data != 0xD0BC );

        drvDB_WriteReg ( 0x3C, 0xE4, 0x2F43 );
    }

    if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        // write file done and check crc
        drvDB_WriteReg ( 0x3C, 0xE4, 0x1380 );
    }

    mdelay ( 10 ); //MCR_CLBK_DEBUG_DELAY ( 10, MCU_LOOP_DELAY_COUNT_MS );

    if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        // polling 0x3CE4 is 0x9432
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
        }
        while ( reg_data != 0x9432 );
    }

    crc_main = crc_main ^ 0xffffffff;
    crc_info = crc_info ^ 0xffffffff;

    if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        // CRC Main from TP
        crc_main_tp = drvDB_ReadReg ( 0x3C, 0x80 );
        crc_main_tp = ( crc_main_tp << 16 ) | drvDB_ReadReg ( 0x3C, 0x82 );

        // CRC Info from TP
        crc_info_tp = drvDB_ReadReg ( 0x3C, 0xA0 );
        crc_info_tp = ( crc_info_tp << 16 ) | drvDB_ReadReg ( 0x3C, 0xA2 );
    }

    printk ( "crc_main=0x%x, crc_info=0x%x, crc_main_tp=0x%x, crc_info_tp=0x%x\n",
               crc_main, crc_info, crc_main_tp, crc_info_tp );

    //drvDB_ExitDBBUS();

    update_pass = 1;

/*
    if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        if ( crc_main_tp != crc_main )
            update_pass = 0;

        if ( crc_info_tp != crc_info )
            update_pass = 0;
    }
*/

    enable_irq ( msg21xx_irq );

    _HalTscrHWReset();

    mdelay( 300 );	

    if ( !update_pass )
    {
        printk ( "update FAILED\n" );
        FwDataCnt = 0;
           return -1;
    }

    printk ( "update OK\n" );
    FwDataCnt = 0;
    
    return 0;
}



static int msg2133_driver_update_fw()
{
	static int first = 0;
    unsigned char i;
    unsigned char dbbus_tx_data[4];
    unsigned char dbbus_rx_data[2] = {0};


    disable_irq_nosync ( msg21xx_irq );


   printk("__ %s __  (%d)\n", __func__, __LINE__);
   
    _HalTscrHWReset();

    mdelay( 300 );
    // Erase TP Flash first
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    mdelay ( 300 );

    // Disable the Watchdog
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x60;
    dbbus_tx_data[3] = 0x55;
    i2c_write_msg2133 ( dbbus_tx_data, 4 );
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x61;
    dbbus_tx_data[3] = 0xAA;
    i2c_write_msg2133 ( dbbus_tx_data, 4 );

    // Stop MCU
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x0F;
    dbbus_tx_data[2] = 0xE6;
    dbbus_tx_data[3] = 0x01;
    i2c_write_msg2133 ( dbbus_tx_data, 4 );

    /////////////////////////
    // Difference between C2 and C3
    /////////////////////////

    //check id
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0xCC;
    i2c_write_msg2133 ( dbbus_tx_data, 3 );
    i2c_read_msg2133 ( &dbbus_rx_data[0], 2 );

    printk ( "dbbus_rx id[0]=0x%x \n", dbbus_rx_data[0] );

    if ( dbbus_rx_data[0] == 2 )
    {
        // check version
        dbbus_tx_data[0] = 0x10;
        dbbus_tx_data[1] = 0x3C;
        dbbus_tx_data[2] = 0xEA;
        i2c_write_msg2133 (  dbbus_tx_data, 3 );
        i2c_read_msg2133 (  &dbbus_rx_data[0], 2 );

        printk ( "dbbus_rx version[0]=0x%x  \n ", dbbus_rx_data[0] );

        if ( dbbus_rx_data[0] == 3 )
        {
	     printk("check go firmware_update_c33\n");
            return firmware_driver_update_c33 ( EMEM_MAIN );
        }
        else
        {
	     printk("check go firmware_update_c32\n");
            return -1;
        }
    }
    else
    {
	 printk("%s: Failed to read firmware data\n", __func__);
	 return -1;
    }

}
static void msg2133_fw_check(struct work_struct *work)
{
    int i,j;
//    u16  fw_version ;	
    u8 fw_data[13] ;
	
	/*fw version*/
	i2c_master_recv ( msg21xx_i2c_client, &fw_data[0], 13 );

	for(i=0;i<13 ;i++)
	  printk("--------------touch fw version data [%d] = 0x%x\n", i , fw_data[i]);
	
	fw_version_now = ((fw_data[10] << 8) | (fw_data[11])) ;
	

//	if (fw_version != 0x0101 )
	if ((fw_version_now < 0x0109 ) || (fw_version_now > 0x1000)){
		if (fw_version_now == 0x0104 ){
			printk("touch fw version do not need to upgrade (touch is PF) \n");
			return;
		}else{
			msg2133_driver_update_fw();
		}
	}else{ 
	       printk("touch fw version do not need to upgrade (touch is GF) \n");
	}
}
#endif


#endif //endif MSG2133_UPDATE


/* 20130220, JimmySu add read msg21xx firmware version*/
static ssize_t msg21xx_firmware_show(struct device *dev, struct device_attribute *attr, char *buf)
{	
//  #cat /sys/bus/i2c/drivers/ms-msg21xx/1-0026/firmware_version
	u16 touch_firmware_version = 0;
	u8 fw_data[13] ;

	
	mdelay(500);
	i2c_master_recv ( msg21xx_i2c_client, &fw_data[0], 13 );
	touch_firmware_version = ((fw_data[10] << 8) | (fw_data[11])) ;
	
	printk("[Touch]:(%s) touch_firmware_version = 0x%x \n",__FUNCTION__, touch_firmware_version);

	return sprintf(buf, "0x%x\n", touch_firmware_version);
}
static DEVICE_ATTR(firmware_version, S_IRUGO|S_IWUSR|S_IWGRP, msg21xx_firmware_show, NULL);

static struct attribute *msg21xx_attributes[] = {
	&dev_attr_firmware_version.attr,
	NULL
};

static const struct attribute_group msg21xx_attr_group = {
	.attrs = msg21xx_attributes,
};
/* 20130220, JimmySu add read msg21xx firmware version*/


/**==========================================================**/

//---------------------------------------------------------------------------
//#define __FIRMWARE_UPDATE__
//#ifdef __FIRMWARE_UPDATE__
#if 0
//#define _FW_UPDATE_C3_

#define FW_ADDR_MSG21XX   (0xC4>>1)
#define FW_ADDR_MSG21XX_TP   (0x4C>>1)
#define FW_UPDATE_ADDR_MSG21XX   (0x92>>1)
#define TP_DEBUG    printk
static  char *fw_version;
static u8 temp[94][1024];
static u8 g_dwiic_info_data[1024];   // Buffer for info data

static int FwDataCnt;
struct class *firmware_class;
struct device *firmware_cmd_dev;

static void HalTscrCReadI2CSeq ( u8 addr, u8* read_data, u16 size )
{
    //according to your platform.
    int rc;

    struct i2c_msg msgs[] =
    {
        {
            .addr = addr,
            .flags = I2C_M_RD,
            .len = size,
            .buf = read_data,
        },
    };

    rc = i2c_transfer ( msg21xx_i2c_client->adapter, msgs, 1 );
    if ( rc < 0 )
    {
        printk ( "HalTscrCReadI2CSeq error %d\n", rc );
    }
}

static void HalTscrCDevWriteI2CSeq ( u8 addr, u8* data, u16 size )
{
    //according to your platform.
    int rc;

    struct i2c_msg msgs[] =
    {
        {
            .addr = addr,
            .flags = 0,
            .len = size,
            .buf = data,
        },
    };
    rc = i2c_transfer ( msg21xx_i2c_client->adapter, msgs, 1 );
    if ( rc < 0 )
    {
        printk ( "HalTscrCDevWriteI2CSeq error %d,addr = %d\n", rc, addr );
    }
}

static void dbbusDWIICEnterSerialDebugMode ( void )
{
    u8 data[5];

    // Enter the Serial Debug Mode
    data[0] = 0x53;
    data[1] = 0x45;
    data[2] = 0x52;
    data[3] = 0x44;
    data[4] = 0x42;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, data, 5 );
}

static void dbbusDWIICStopMCU ( void )
{
    u8 data[1];

    // Stop the MCU
    data[0] = 0x37;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, data, 1 );
}

static void dbbusDWIICIICUseBus ( void )
{
    u8 data[1];

    // IIC Use Bus
    data[0] = 0x35;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, data, 1 );
}

static void dbbusDWIICIICReshape ( void )
{
    u8 data[1];

    // IIC Re-shape
    data[0] = 0x71;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, data, 1 );
}

static void dbbusDWIICIICNotUseBus ( void )
{
    u8 data[1];

    // IIC Not Use Bus
    data[0] = 0x34;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, data, 1 );
}

static void dbbusDWIICNotStopMCU ( void )
{
    u8 data[1];

    // Not Stop the MCU
    data[0] = 0x36;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, data, 1 );
}

static void dbbusDWIICExitSerialDebugMode ( void )
{
    u8 data[1];

    // Exit the Serial Debug Mode
    data[0] = 0x45;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, data, 1 );

    // Delay some interval to guard the next transaction
    //udelay ( 200 );        // delay about 0.2ms
}

static void drvISP_EntryIspMode ( void )
{
    u8 bWriteData[5] =
    {
        0x4D, 0x53, 0x54, 0x41, 0x52
    };

    HalTscrCDevWriteI2CSeq ( FW_UPDATE_ADDR_MSG21XX, bWriteData, 5 );
}

static u8 drvISP_Read ( u8 n, u8* pDataToRead ) //First it needs send 0x11 to notify we want to get flash data back.
{
    u8 Read_cmd = 0x11;
    unsigned char dbbus_rx_data[2] = {0xFF, 0xFF};
    HalTscrCDevWriteI2CSeq ( FW_UPDATE_ADDR_MSG21XX, &Read_cmd, 1 );
//    msctpc_LoopDelay ( 10 );        // delay about 1000us*****
	mdelay(1);
    if ( n == 1 )
    {
        HalTscrCReadI2CSeq ( FW_UPDATE_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );

        // Ideally, the obtained dbbus_rx_data[0~1] stands for the following meaning:
        //  dbbus_rx_data[0]  |  dbbus_rx_data[1]  | status
        // -------------------+--------------------+--------
        //       0x00         |       0x00         |  0x00
        // -------------------+--------------------+--------
        //       0x??         |       0x00         |  0x??
        // -------------------+--------------------+--------
        //       0x00         |       0x??         |  0x??
        //
        // Therefore, we build this field patch to return the status to *pDataToRead.
        *pDataToRead = ( ( dbbus_rx_data[0] >= dbbus_rx_data[1] ) ? \
                         dbbus_rx_data[0]  : dbbus_rx_data[1] );
    }
    else
    {
        HalTscrCReadI2CSeq ( FW_UPDATE_ADDR_MSG21XX, pDataToRead, n );
    }

    return 0;
}

static void drvISP_WriteEnable ( void )
{
    u8 bWriteData[2] =
    {
        0x10, 0x06
    };
    u8 bWriteData1 = 0x12;
    HalTscrCDevWriteI2CSeq ( FW_UPDATE_ADDR_MSG21XX, bWriteData, 2 );
    HalTscrCDevWriteI2CSeq ( FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1 );
}

static void drvISP_ExitIspMode ( void )
{
    u8 bWriteData = 0x24;
    HalTscrCDevWriteI2CSeq ( FW_UPDATE_ADDR_MSG21XX, &bWriteData, 1 );
}

static u8 drvISP_ReadStatus ( void )
{
    u8 bReadData = 0;
    u8 bWriteData[2] =
    {
        0x10, 0x05
    };
    u8 bWriteData1 = 0x12;

    HalTscrCDevWriteI2CSeq ( FW_UPDATE_ADDR_MSG21XX, bWriteData, 2 );
//    msctpc_LoopDelay ( 1 );        // delay about 100us*****
    udelay(100);
    drvISP_Read ( 1, &bReadData );
    HalTscrCDevWriteI2CSeq ( FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1 );
    return bReadData;
}

static void drvISP_ChipErase()
{
    u8 bWriteData[5] = { 0x00, 0x00, 0x00, 0x00, 0x00 };
    u8 bWriteData1 = 0x12;
    u32 timeOutCount = 0;
    drvISP_WriteEnable();

    //Enable write status register
    bWriteData[0] = 0x10;
    bWriteData[1] = 0x50;
    HalTscrCDevWriteI2CSeq ( FW_UPDATE_ADDR_MSG21XX, bWriteData, 2 );
    HalTscrCDevWriteI2CSeq ( FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1 );

    //Write Status
    bWriteData[0] = 0x10;
    bWriteData[1] = 0x01;
    bWriteData[2] = 0x00;
    HalTscrCDevWriteI2CSeq ( FW_UPDATE_ADDR_MSG21XX, bWriteData, 3 );
    HalTscrCDevWriteI2CSeq ( FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1 );

    //Write disable
    bWriteData[0] = 0x10;
    bWriteData[1] = 0x04;
    HalTscrCDevWriteI2CSeq ( FW_UPDATE_ADDR_MSG21XX, bWriteData, 2 );
    HalTscrCDevWriteI2CSeq ( FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1 );
//    msctpc_LoopDelay ( 1 );        // delay about 100us*****
    udelay(100);
    timeOutCount = 0;
    while ( ( drvISP_ReadStatus() & 0x01 ) == 0x01 )
    {
        timeOutCount++;
        if ( timeOutCount >= 100000 ) break; /* around 1 sec timeout */
    }
    drvISP_WriteEnable();

    bWriteData[0] = 0x10;
    bWriteData[1] = 0xC7;

    HalTscrCDevWriteI2CSeq ( FW_UPDATE_ADDR_MSG21XX, bWriteData, 2 );
    HalTscrCDevWriteI2CSeq ( FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1 );
//    msctpc_LoopDelay ( 1 );        // delay about 100us*****
    udelay(100);
    timeOutCount = 0;
    while ( ( drvISP_ReadStatus() & 0x01 ) == 0x01 )
    {
        timeOutCount++;
        if ( timeOutCount >= 500000 ) break; /* around 5 sec timeout */
    }
}

static void drvISP_Program ( u16 k, u8* pDataToWrite )
{
    u16 i = 0;
    u16 j = 0;
    //u16 n = 0;
    u8 TX_data[133];
    u8 bWriteData1 = 0x12;
    u32 addr = k * 1024;
    u32 timeOutCount = 0;
    for ( j = 0; j < 8; j++ ) //128*8 cycle
    {
        TX_data[0] = 0x10;
        TX_data[1] = 0x02;// Page Program CMD
        TX_data[2] = ( addr + 128 * j ) >> 16;
        TX_data[3] = ( addr + 128 * j ) >> 8;
        TX_data[4] = ( addr + 128 * j );
        for ( i = 0; i < 128; i++ )
        {
            TX_data[5 + i] = pDataToWrite[j * 128 + i];
        }
//        msctpc_LoopDelay ( 1 );        // delay about 100us*****
        udelay(100);

        timeOutCount = 0;
        while ( ( drvISP_ReadStatus() & 0x01 ) == 0x01 )
        {
            timeOutCount++;
            if ( timeOutCount >= 100000 ) break; /* around 1 sec timeout */
        }

        drvISP_WriteEnable();
        HalTscrCDevWriteI2CSeq ( FW_UPDATE_ADDR_MSG21XX, TX_data, 133 ); //write 133 byte per cycle
        HalTscrCDevWriteI2CSeq ( FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1 );
    }
}

static void drvISP_Verify ( u16 k, u8* pDataToVerify )
{
    u16 i = 0, j = 0;
    u8 bWriteData[5] =
    {
        0x10, 0x03, 0, 0, 0
    };
    u8 RX_data[256];
    u8 bWriteData1 = 0x12;
    u32 addr = k * 1024;
    u8 index = 0;
    u32 timeOutCount;
    for ( j = 0; j < 8; j++ ) //128*8 cycle
    {
        bWriteData[2] = ( u8 ) ( ( addr + j * 128 ) >> 16 );
        bWriteData[3] = ( u8 ) ( ( addr + j * 128 ) >> 8 );
        bWriteData[4] = ( u8 ) ( addr + j * 128 );
//        msctpc_LoopDelay ( 1 );        // delay about 100us*****
        udelay(100);

        timeOutCount = 0;
        while ( ( drvISP_ReadStatus() & 0x01 ) == 0x01 )
        {
            timeOutCount++;
            if ( timeOutCount >= 100000 ) break; /* around 1 sec timeout */
        }

        HalTscrCDevWriteI2CSeq ( FW_UPDATE_ADDR_MSG21XX, bWriteData, 5 ); //write read flash addr
//        msctpc_LoopDelay ( 1 );        // delay about 100us*****
        udelay(100);
        drvISP_Read ( 128, RX_data );
        HalTscrCDevWriteI2CSeq ( FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1 ); //cmd end
        for ( i = 0; i < 128; i++ ) //log out if verify error
        {
            if ( ( RX_data[i] != 0 ) && index < 10 )
            {
                //TP_DEBUG("j=%d,RX_data[%d]=0x%x\n",j,i,RX_data[i]);
                index++;
            }
            if ( RX_data[i] != pDataToVerify[128 * j + i] )
            {
                TP_DEBUG ( "k=%d,j=%d,i=%d===============Update Firmware Error================", k, j, i );
            }
        }
    }
}

static ssize_t firmware_update_show ( struct device *dev,
                                      struct device_attribute *attr, char *buf )
{
    return sprintf ( buf, "%s\n", fw_version );
}

static void _HalTscrHWReset ( void )
{
    gpio_direction_output ( MSG21XX_RESET_GPIO, 1 );
    gpio_set_value ( MSG21XX_RESET_GPIO, 1 );
    gpio_set_value ( MSG21XX_RESET_GPIO, 0 );
    mdelay ( 10 ); /* Note that the RST must be in LOW 10ms at least */
    gpio_set_value ( MSG21XX_RESET_GPIO, 1 );
    /* Enable the interrupt service thread/routine for INT after 50ms */
    mdelay ( 50 );
}

static ssize_t firmware_update_c2 ( struct device *dev,
                                    struct device_attribute *attr, const char *buf, size_t size )
{
    u8 i;
    u8 dbbus_tx_data[4];
    unsigned char dbbus_rx_data[2] = {0};

    // set FRO to 50M
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x11;
    dbbus_tx_data[2] = 0xE2;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
    dbbus_rx_data[0] = 0;
    dbbus_rx_data[1] = 0;
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
    TP_DEBUG ( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
    dbbus_tx_data[3] = dbbus_rx_data[0] & 0xF7;  //Clear Bit 3
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // set MCU clock,SPI clock =FRO
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x22;
    dbbus_tx_data[3] = 0x00;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x23;
    dbbus_tx_data[3] = 0x00;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // Enable slave's ISP ECO mode
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x08;
    dbbus_tx_data[2] = 0x0c;
    dbbus_tx_data[3] = 0x08;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // Enable SPI Pad
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x02;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
    TP_DEBUG ( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
    dbbus_tx_data[3] = ( dbbus_rx_data[0] | 0x20 ); //Set Bit 5
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // WP overwrite
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x0E;
    dbbus_tx_data[3] = 0x02;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // set pin high
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x10;
    dbbus_tx_data[3] = 0x08;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    dbbusDWIICIICNotUseBus();
    dbbusDWIICNotStopMCU();
    dbbusDWIICExitSerialDebugMode();

    drvISP_EntryIspMode();
    drvISP_ChipErase();
    _HalTscrHWReset();
    mdelay ( 300 );

    // Program and Verify
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();

    // Disable the Watchdog
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x60;
    dbbus_tx_data[3] = 0x55;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x61;
    dbbus_tx_data[3] = 0xAA;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    //Stop MCU
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x0F;
    dbbus_tx_data[2] = 0xE6;
    dbbus_tx_data[3] = 0x01;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // set FRO to 50M
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x11;
    dbbus_tx_data[2] = 0xE2;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
    dbbus_rx_data[0] = 0;
    dbbus_rx_data[1] = 0;
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
    TP_DEBUG ( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
    dbbus_tx_data[3] = dbbus_rx_data[0] & 0xF7;  //Clear Bit 3
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // set MCU clock,SPI clock =FRO
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x22;
    dbbus_tx_data[3] = 0x00;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x23;
    dbbus_tx_data[3] = 0x00;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // Enable slave's ISP ECO mode
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x08;
    dbbus_tx_data[2] = 0x0c;
    dbbus_tx_data[3] = 0x08;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // Enable SPI Pad
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x02;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
    TP_DEBUG ( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
    dbbus_tx_data[3] = ( dbbus_rx_data[0] | 0x20 ); //Set Bit 5
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // WP overwrite
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x0E;
    dbbus_tx_data[3] = 0x02;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // set pin high
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x10;
    dbbus_tx_data[3] = 0x08;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    dbbusDWIICIICNotUseBus();
    dbbusDWIICNotStopMCU();
    dbbusDWIICExitSerialDebugMode();

    ///////////////////////////////////////
    // Start to load firmware
    ///////////////////////////////////////
    drvISP_EntryIspMode();

    for ( i = 0; i < 94; i++ ) // total  94 KB : 1 byte per R/W
    {
        drvISP_Program ( i, temp[i] ); // program to slave's flash
        drvISP_Verify ( i, temp[i] ); //verify data
    }
    TP_DEBUG ( "update OK\n" );
    drvISP_ExitIspMode();
    FwDataCnt = 0;
    return size;
}

static void drvDB_WriteReg ( u8 bank, u8 addr, u16 data )
{
    u8 tx_data[5] = {0x10, bank, addr, data & 0xFF, data >> 8};
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, tx_data, 5 );
}

static void drvDB_WriteReg8Bit ( u8 bank, u8 addr, u8 data )
{
    u8 tx_data[4] = {0x10, bank, addr, data};
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, tx_data, 4 );
}

static unsigned short drvDB_ReadReg ( u8 bank, u8 addr )
{
    u8 tx_data[3] = {0x10, bank, addr};
    u8 rx_data[2] = {0};

    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, tx_data, 3 );
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &rx_data[0], 2 );
    return ( rx_data[1] << 8 | rx_data[0] );
}

static u32 Reflect ( u32 ref, char ch ) //unsigned int Reflect(unsigned int ref, char ch)
{
    u32 value = 0;
    u32 i = 0;

    for ( i = 1; i < ( ch + 1 ); i++ )
    {
        if ( ref & 1 )
        {
            value |= 1 << ( ch - i );
        }
        ref >>= 1;
    }
    return value;
}

static void Init_CRC32_Table ( u32 *crc32_table )
{
    u32 magicnumber = 0x04c11db7;
    u32 i = 0, j;

    for ( i = 0; i <= 0xFF; i++ )
    {
        crc32_table[i] = Reflect ( i, 8 ) << 24;
        for ( j = 0; j < 8; j++ )
        {
            crc32_table[i] = ( crc32_table[i] << 1 ) ^ ( crc32_table[i] & ( 0x80000000L ) ? magicnumber : 0 );
        }
        crc32_table[i] = Reflect ( crc32_table[i], 32 );
    }
}

u32 Get_CRC ( u32 text, u32 prevCRC, u32 *crc32_table )
{
    u32  ulCRC = prevCRC;
    {
        ulCRC = ( ulCRC >> 8 ) ^ crc32_table[ ( ulCRC & 0xFF ) ^ text];
    }
    return ulCRC ;
}


static int drvTP_erase_emem_c32 ( void )
{
    /////////////////////////
    //Erase  all
    /////////////////////////
    
    //enter gpio mode
    drvDB_WriteReg ( 0x16, 0x1E, 0xBEAF );

    // before gpio mode, set the control pin as the orginal status
    drvDB_WriteReg ( 0x16, 0x08, 0x0000 );
    drvDB_WriteReg8Bit ( 0x16, 0x0E, 0x10 );
    mdelay ( 10 ); //MCR_CLBK_DEBUG_DELAY ( 10, MCU_LOOP_DELAY_COUNT_MS );

    // ptrim = 1, h'04[2]
    drvDB_WriteReg8Bit ( 0x16, 0x08, 0x04 );
    drvDB_WriteReg8Bit ( 0x16, 0x0E, 0x10 );
    mdelay ( 10 ); //MCR_CLBK_DEBUG_DELAY ( 10, MCU_LOOP_DELAY_COUNT_MS );

    // ptm = 6, h'04[12:14] = b'110
    drvDB_WriteReg8Bit ( 0x16, 0x09, 0x60 );
    drvDB_WriteReg8Bit ( 0x16, 0x0E, 0x10 );

    // pmasi = 1, h'04[6]
    drvDB_WriteReg8Bit ( 0x16, 0x08, 0x44 );
    // pce = 1, h'04[11]
    drvDB_WriteReg8Bit ( 0x16, 0x09, 0x68 );
    // perase = 1, h'04[7]
    drvDB_WriteReg8Bit ( 0x16, 0x08, 0xC4 );
    // pnvstr = 1, h'04[5]
    drvDB_WriteReg8Bit ( 0x16, 0x08, 0xE4 );
    // pwe = 1, h'04[9]
    drvDB_WriteReg8Bit ( 0x16, 0x09, 0x6A );
    // trigger gpio load
    drvDB_WriteReg8Bit ( 0x16, 0x0E, 0x10 );

    return ( 1 );
}

static ssize_t firmware_update_c32 ( struct device *dev, struct device_attribute *attr,
                                     const char *buf, size_t size,  EMEM_TYPE_t emem_type )
{
    u8  dbbus_tx_data[4];
    u8  dbbus_rx_data[2] = {0};
    u8  Fmr_Loader[1024];   // Buffer for slave's firmware

    u32 i, j;
    u32 crc_main, crc_main_tp;
    u32 crc_info, crc_info_tp;
    u32 crc_tab[256];
    u16 reg_data = 0;

    crc_main = 0xffffffff;
    crc_info = 0xffffffff;

#if 1
    /////////////////////////
    // Erase  all
    /////////////////////////
    drvTP_erase_emem_c32();
    mdelay ( 1000 ); //MCR_CLBK_DEBUG_DELAY ( 1000, MCU_LOOP_DELAY_COUNT_MS );

    //ResetSlave();
    _HalTscrHWReset();
    //drvDB_EnterDBBUS();
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    mdelay ( 300 );

    // Reset Watchdog
    drvDB_WriteReg8Bit ( 0x3C, 0x60, 0x55 );
    drvDB_WriteReg8Bit ( 0x3C, 0x61, 0xAA );

    /////////////////////////
    // Program
    /////////////////////////

    //polling 0x3CE4 is 0x1C70
    do
    {
        reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
    }
    while ( reg_data != 0x1C70 );


    drvDB_WriteReg ( 0x3C, 0xE4, 0xE38F );  // for all-blocks

    //polling 0x3CE4 is 0x2F43
    do
    {
        reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
    }
    while ( reg_data != 0x2F43 );


    //calculate CRC 32
    Init_CRC32_Table ( &crc_tab[0] );

    for ( i = 0; i < 33; i++ ) // total  33 KB : 2 byte per R/W
    {
        if ( i < 32 )   //emem_main
        {
            if ( i == 31 )
            {
                temp[i][1014] = 0x5A; //Fmr_Loader[1014]=0x5A;
                temp[i][1015] = 0xA5; //Fmr_Loader[1015]=0xA5;

                for ( j = 0; j < 1016; j++ )
                {
                    //crc_main=Get_CRC(Fmr_Loader[j],crc_main,&crc_tab[0]);
                    crc_main = Get_CRC ( temp[i][j], crc_main, &crc_tab[0] );
                }
            }
            else
            {
                for ( j = 0; j < 1024; j++ )
                {
                    //crc_main=Get_CRC(Fmr_Loader[j],crc_main,&crc_tab[0]);
                    crc_main = Get_CRC ( temp[i][j], crc_main, &crc_tab[0] );
                }
            }
        }
        else  // emem_info
        {
            for ( j = 0; j < 1024; j++ )
            {
                //crc_info=Get_CRC(Fmr_Loader[j],crc_info,&crc_tab[0]);
                crc_info = Get_CRC ( temp[i][j], crc_info, &crc_tab[0] );
            }
        }

        //drvDWIIC_MasterTransmit( DWIIC_MODE_DWIIC_ID, 1024, Fmr_Loader );
        HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX_TP, temp[i], 1024 );

        // polling 0x3CE4 is 0xD0BC
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
        }
        while ( reg_data != 0xD0BC );

        drvDB_WriteReg ( 0x3C, 0xE4, 0x2F43 );
    }

    //write file done
    drvDB_WriteReg ( 0x3C, 0xE4, 0x1380 );

    mdelay ( 10 ); //MCR_CLBK_DEBUG_DELAY ( 10, MCU_LOOP_DELAY_COUNT_MS );
    // polling 0x3CE4 is 0x9432
    do
    {
        reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
    }
    while ( reg_data != 0x9432 );

    crc_main = crc_main ^ 0xffffffff;
    crc_info = crc_info ^ 0xffffffff;

    // CRC Main from TP
    crc_main_tp = drvDB_ReadReg ( 0x3C, 0x80 );
    crc_main_tp = ( crc_main_tp << 16 ) | drvDB_ReadReg ( 0x3C, 0x82 );
 
    //CRC Info from TP
    crc_info_tp = drvDB_ReadReg ( 0x3C, 0xA0 );
    crc_info_tp = ( crc_info_tp << 16 ) | drvDB_ReadReg ( 0x3C, 0xA2 );

    TP_DEBUG ( "crc_main=0x%x, crc_info=0x%x, crc_main_tp=0x%x, crc_info_tp=0x%x\n",
               crc_main, crc_info, crc_main_tp, crc_info_tp );

    //drvDB_ExitDBBUS();
    if ( ( crc_main_tp != crc_main ) || ( crc_info_tp != crc_info ) )
    {
        TP_DEBUG ( "update FAILED\n" );
        FwDataCnt = 0;
        return ( 0 );
    }

    TP_DEBUG ( "update OK\n" );
    FwDataCnt = 0;
    return size;

#endif
}

static int drvTP_erase_emem_c33 ( EMEM_TYPE_t emem_type )
{
    // stop mcu
    drvDB_WriteReg ( 0x0F, 0xE6, 0x0001 );

    //disable watch dog
    drvDB_WriteReg8Bit ( 0x3C, 0x60, 0x55 );
    drvDB_WriteReg8Bit ( 0x3C, 0x61, 0xAA );

    // set PROGRAM password
    drvDB_WriteReg8Bit ( 0x16, 0x1A, 0xBA );
    drvDB_WriteReg8Bit ( 0x16, 0x1B, 0xAB );

    //proto.MstarWriteReg(F1.loopDevice, 0x1618, 0x80);
    drvDB_WriteReg8Bit ( 0x16, 0x18, 0x80 );

    if ( emem_type == EMEM_ALL )
    {
        drvDB_WriteReg8Bit ( 0x16, 0x08, 0x10 ); //mark
    }

    drvDB_WriteReg8Bit ( 0x16, 0x18, 0x40 );
    mdelay ( 10 );

    drvDB_WriteReg8Bit ( 0x16, 0x18, 0x80 );

    // erase trigger
    if ( emem_type == EMEM_MAIN )
    {
        drvDB_WriteReg8Bit ( 0x16, 0x0E, 0x04 ); //erase main
    }
    else
    {
        drvDB_WriteReg8Bit ( 0x16, 0x0E, 0x08 ); //erase all block
    }

    return ( 1 );
}

static int drvTP_read_emem_dbbus_c33 ( EMEM_TYPE_t emem_type, u16 addr, size_t size, u8 *p, bool_t set_pce_high )
{
    u32 i;

    // Set the starting address ( must before enabling burst mode and enter riu mode )
    drvDB_WriteReg ( 0x16, 0x00, addr );

    // Enable the burst mode ( must before enter riu mode )
    drvDB_WriteReg ( 0x16, 0x0C, drvDB_ReadReg ( 0x16, 0x0C ) | 0x0001 );

    // Set the RIU password
    drvDB_WriteReg ( 0x16, 0x1A, 0xABBA );

    // Enable the information block if pifren is HIGH
    if ( emem_type == EMEM_INFO )
    {
        // Clear the PCE
        drvDB_WriteReg ( 0x16, 0x18, drvDB_ReadReg ( 0x16, 0x18 ) | 0x0080 );
        mdelay ( 10 );

        // Set the PIFREN to be HIGH
        drvDB_WriteReg ( 0x16, 0x08, 0x0010 );
    }

    // Set the PCE to be HIGH
    drvDB_WriteReg ( 0x16, 0x18, drvDB_ReadReg ( 0x16, 0x18 ) | 0x0040 );
    mdelay ( 10 );

    // Wait pce becomes 1 ( read data ready )
    while ( ( drvDB_ReadReg ( 0x16, 0x10 ) & 0x0004 ) != 0x0004 );

    for ( i = 0; i < size; i += 4 )
    {
        // Fire the FASTREAD command
        drvDB_WriteReg ( 0x16, 0x0E, drvDB_ReadReg ( 0x16, 0x0E ) | 0x0001 );

        // Wait the operation is done
        while ( ( drvDB_ReadReg ( 0x16, 0x10 ) & 0x0001 ) != 0x0001 );

        p[i + 0] = drvDB_ReadReg ( 0x16, 0x04 ) & 0xFF;
        p[i + 1] = ( drvDB_ReadReg ( 0x16, 0x04 ) >> 8 ) & 0xFF;
        p[i + 2] = drvDB_ReadReg ( 0x16, 0x06 ) & 0xFF;
        p[i + 3] = ( drvDB_ReadReg ( 0x16, 0x06 ) >> 8 ) & 0xFF;
    }

    // Disable the burst mode
    drvDB_WriteReg ( 0x16, 0x0C, drvDB_ReadReg ( 0x16, 0x0C ) & ( ~0x0001 ) );

    // Clear the starting address
    drvDB_WriteReg ( 0x16, 0x00, 0x0000 );

    //Always return to main block
    if ( emem_type == EMEM_INFO )
    {
        // Clear the PCE before change block
        drvDB_WriteReg ( 0x16, 0x18, drvDB_ReadReg ( 0x16, 0x18 ) | 0x0080 );
        mdelay ( 10 );
        // Set the PIFREN to be LOW
        drvDB_WriteReg ( 0x16, 0x08, drvDB_ReadReg ( 0x16, 0x08 ) & ( ~0x0010 ) );

        drvDB_WriteReg ( 0x16, 0x18, drvDB_ReadReg ( 0x16, 0x18 ) | 0x0040 );
        while ( ( drvDB_ReadReg ( 0x16, 0x10 ) & 0x0004 ) != 0x0004 );
    }

    // Clear the RIU password
    drvDB_WriteReg ( 0x16, 0x1A, 0x0000 );

    if ( set_pce_high )
    {
        // Set the PCE to be HIGH before jumping back to e-flash codes
        drvDB_WriteReg ( 0x16, 0x18, drvDB_ReadReg ( 0x16, 0x18 ) | 0x0040 );
        while ( ( drvDB_ReadReg ( 0x16, 0x10 ) & 0x0004 ) != 0x0004 );
    }

    return ( 1 );
}

static int drvTP_read_info_dwiic_c33 ( void )
{
    u8  dwiic_tx_data[5];

    //drvDB_EnterDBBUS();
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    mdelay ( 300 );

    // Stop Watchdog
    drvDB_WriteReg8Bit ( 0x3C, 0x60, 0x55 );
    drvDB_WriteReg8Bit ( 0x3C, 0x61, 0xAA );

    drvDB_WriteReg ( 0x3C, 0xE4, 0xA4AB );

    // TP SW reset
    drvDB_WriteReg ( 0x1E, 0x02, 0x829F );

    mdelay ( 50 );

    dwiic_tx_data[0] = 0x72;
    dwiic_tx_data[1] = 0x80;
    dwiic_tx_data[2] = 0x00;
    dwiic_tx_data[3] = 0x04;
    dwiic_tx_data[4] = 0x00;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX_TP , dwiic_tx_data, 5 );

    mdelay ( 50 );

    // recive info data
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX_TP, &g_dwiic_info_data[0], 1024 );

    return ( 1 );
}

static int drvTP_info_updata_C33 ( u16 start_index, u8 *data, u16 size )
{
    // size != 0, start_index+size !> 1024
    u16 i;
    
    for ( i = 0; i < size; i++ )
    {
        g_dwiic_info_data[start_index] = * ( data + i );
        start_index++;
    }
    
    return ( 1 );
}


static ssize_t firmware_update_c33 ( struct device *dev, struct device_attribute *attr,
                                     const char *buf, size_t size, EMEM_TYPE_t emem_type )
{
    u8  dbbus_tx_data[4];
    u8  dbbus_rx_data[2] = {0};
    u8  life_counter[2];
    u32 i, j;
    u32 crc_main, crc_main_tp;
    u32 crc_info, crc_info_tp;
    u32 crc_tab[256];

    int update_pass = 1;
    u16 reg_data = 0;

    crc_main = 0xffffffff;
    crc_info = 0xffffffff;

    drvTP_read_info_dwiic_c33();

    if ( g_dwiic_info_data[0] == 'M' && g_dwiic_info_data[1] == 'S' && g_dwiic_info_data[2] == 'T' && g_dwiic_info_data[3] == 'A' && g_dwiic_info_data[4] == 'R' && g_dwiic_info_data[5] == 'T' && g_dwiic_info_data[6] == 'P' && g_dwiic_info_data[7] == 'C' )
    {
        // updata FW Version
        drvTP_info_updata_C33 ( 8, temp[32][8], 4 );

        // updata life counter
        life_counter[0] = ( ( g_dwiic_info_data[12] << 8 + g_dwiic_info_data[13] + 1 ) >> 8 ) & 0xFF;
        life_counter[1] = ( g_dwiic_info_data[12] << 8 + g_dwiic_info_data[13] + 1 ) & 0xFF;
        drvTP_info_updata_C33 ( 10, life_counter, 2 );

        drvDB_WriteReg ( 0x3C, 0xE4, 0x78C5 );

        // TP SW reset
        drvDB_WriteReg ( 0x1E, 0x02, 0x829F );

        mdelay ( 50 );

        //polling 0x3CE4 is 0x2F43
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
        }
        while ( reg_data != 0x2F43 );

        // transmit lk info data
        HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX_TP , g_dwiic_info_data, 1024 );

        //polling 0x3CE4 is 0xD0BC
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
        }
        while ( reg_data != 0xD0BC );

    }


    //erase main
    drvTP_erase_emem_c33 ( EMEM_MAIN );
    mdelay ( 1000 );

    //ResetSlave();
    _HalTscrHWReset();

    //drvDB_EnterDBBUS();
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    mdelay ( 300 );

    /////////////////////////
    // Program
    /////////////////////////

    //polling 0x3CE4 is 0x1C70
    if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
        }
        while ( reg_data != 0x1C70 );
    }

    switch ( emem_type )
    {
        case EMEM_ALL:
            drvDB_WriteReg ( 0x3C, 0xE4, 0xE38F );  // for all-blocks
            break;
        case EMEM_MAIN:
            drvDB_WriteReg ( 0x3C, 0xE4, 0x7731 );  // for main block
            break;
        case EMEM_INFO:
            drvDB_WriteReg ( 0x3C, 0xE4, 0x7731 );  // for info block


            drvDB_WriteReg8Bit ( 0x0F, 0xE6, 0x01 );

            drvDB_WriteReg8Bit ( 0x3C, 0xE4, 0xC5 ); //
            drvDB_WriteReg8Bit ( 0x3C, 0xE5, 0x78 ); //

            drvDB_WriteReg8Bit ( 0x1E, 0x04, 0x9F );
            drvDB_WriteReg8Bit ( 0x1E, 0x05, 0x82 );

            drvDB_WriteReg8Bit ( 0x0F, 0xE6, 0x00 );
            mdelay ( 100 );
            break;
    }

    // polling 0x3CE4 is 0x2F43
    do
    {
        reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
    }
    while ( reg_data != 0x2F43 );


    // calculate CRC 32
    Init_CRC32_Table ( &crc_tab[0] );


    for ( i = 0; i < 33; i++ ) // total  33 KB : 2 byte per R/W
    {
        if ( emem_type == EMEM_INFO ) i = 32;

        if ( i < 32 )   //emem_main
        {
            if ( i == 31 )
            {
                temp[i][1014] = 0x5A; //Fmr_Loader[1014]=0x5A;
                temp[i][1015] = 0xA5; //Fmr_Loader[1015]=0xA5;

                for ( j = 0; j < 1016; j++ )
                {
                    //crc_main=Get_CRC(Fmr_Loader[j],crc_main,&crc_tab[0]);
                    crc_main = Get_CRC ( temp[i][j], crc_main, &crc_tab[0] );
                }
            }
            else
            {
                for ( j = 0; j < 1024; j++ )
                {
                    //crc_main=Get_CRC(Fmr_Loader[j],crc_main,&crc_tab[0]);
                    crc_main = Get_CRC ( temp[i][j], crc_main, &crc_tab[0] );
                }
            }
        }
        else  //emem_info
        {
            for ( j = 0; j < 1024; j++ )
            {
                //crc_info=Get_CRC(Fmr_Loader[j],crc_info,&crc_tab[0]);
                crc_info = Get_CRC ( g_dwiic_info_data[j], crc_info, &crc_tab[0] );
            }
            if ( emem_type == EMEM_MAIN ) break;
        }

        //drvDWIIC_MasterTransmit( DWIIC_MODE_DWIIC_ID, 1024, Fmr_Loader );
        HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX_TP, temp[i], 1024 );

        // polling 0x3CE4 is 0xD0BC
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
        }
        while ( reg_data != 0xD0BC );

        drvDB_WriteReg ( 0x3C, 0xE4, 0x2F43 );
    }

    if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        // write file done and check crc
        drvDB_WriteReg ( 0x3C, 0xE4, 0x1380 );
    }

    mdelay ( 10 ); //MCR_CLBK_DEBUG_DELAY ( 10, MCU_LOOP_DELAY_COUNT_MS );

    if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        // polling 0x3CE4 is 0x9432
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
        }
        while ( reg_data != 0x9432 );
    }

    crc_main = crc_main ^ 0xffffffff;
    crc_info = crc_info ^ 0xffffffff;

    if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        // CRC Main from TP
        crc_main_tp = drvDB_ReadReg ( 0x3C, 0x80 );
        crc_main_tp = ( crc_main_tp << 16 ) | drvDB_ReadReg ( 0x3C, 0x82 );

        // CRC Info from TP
        crc_info_tp = drvDB_ReadReg ( 0x3C, 0xA0 );
        crc_info_tp = ( crc_info_tp << 16 ) | drvDB_ReadReg ( 0x3C, 0xA2 );
    }

    TP_DEBUG ( "crc_main=0x%x, crc_info=0x%x, crc_main_tp=0x%x, crc_info_tp=0x%x\n",
               crc_main, crc_info, crc_main_tp, crc_info_tp );

    //drvDB_ExitDBBUS();

    update_pass = 1;
                  if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        if ( crc_main_tp != crc_main )
            update_pass = 0;

        if ( crc_info_tp != crc_info )
            update_pass = 0;
    }

    if ( !update_pass )
    {
        TP_DEBUG ( "update FAILED\n" );
        FwDataCnt = 0;
        return ( 0 );
    }

    TP_DEBUG ( "update OK\n" );
    FwDataCnt = 0;
    
    return size;
}

#ifdef _FW_UPDATE_C3_
static ssize_t firmware_update_store ( struct device *dev,
                                       struct device_attribute *attr, const char *buf, size_t size )
{
    u8 i;
    u8 dbbus_tx_data[4];
    unsigned char dbbus_rx_data[2] = {0};


    _HalTscrHWReset();

    // Erase TP Flash first
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    mdelay ( 300 );

    // Disable the Watchdog
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x60;
    dbbus_tx_data[3] = 0x55;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x61;
    dbbus_tx_data[3] = 0xAA;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // Stop MCU
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x0F;
    dbbus_tx_data[2] = 0xE6;
    dbbus_tx_data[3] = 0x01;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );


    /////////////////////////
    // Difference between C2 and C3
    /////////////////////////

    //check id
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0xCC;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
    TP_DEBUG ( "dbbus_rx id[0]=0x%x", dbbus_rx_data[0] );

    if ( dbbus_rx_data[0] == 2 )
    {
        // check version
        dbbus_tx_data[0] = 0x10;
        dbbus_tx_data[1] = 0x3C;
        dbbus_tx_data[2] = 0xEA;
        HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
        HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
        TP_DEBUG ( "dbbus_rx version[0]=0x%x", dbbus_rx_data[0] );

        if ( dbbus_rx_data[0] == 3 )
        {
            return firmware_update_c33 ( dev, attr, buf, size, EMEM_MAIN );
        }
        else
        {
            return firmware_update_c32 ( dev, attr, buf, size, EMEM_ALL );
        }
    }
    else
    {
        return firmware_update_c2 ( dev, attr, buf, size );
    }

}

#else

static ssize_t firmware_update_store ( struct device *dev,
                                       struct device_attribute *attr, const char *buf, size_t size )
{
    u8 i;
    u8 dbbus_tx_data[4];
    unsigned char dbbus_rx_data[2] = {0};


    _HalTscrHWReset();

    // 1. Erase TP Flash first
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    mdelay ( 300 );


    // Disable the Watchdog
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x60;
    dbbus_tx_data[3] = 0x55;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x61;
    dbbus_tx_data[3] = 0xAA;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // Stop MCU
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x0F;
    dbbus_tx_data[2] = 0xE6;
    dbbus_tx_data[3] = 0x01;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );


    // set FRO to 50M
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x11;
    dbbus_tx_data[2] = 0xE2;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
    dbbus_rx_data[0] = 0;
    dbbus_rx_data[1] = 0;
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
    TP_DEBUG ( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
    dbbus_tx_data[3] = dbbus_rx_data[0] & 0xF7;  //Clear Bit 3
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );


    // set MCU clock,SPI clock =FRO
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x22;
    dbbus_tx_data[3] = 0x00;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x23;
    dbbus_tx_data[3] = 0x00;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );


    // Enable slave's ISP ECO mode
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x08;
    dbbus_tx_data[2] = 0x0c;
    dbbus_tx_data[3] = 0x08;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // Enable SPI Pad
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x02;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
    TP_DEBUG ( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
    dbbus_tx_data[3] = ( dbbus_rx_data[0] | 0x20 ); //Set Bit 5
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );


    // WP overwrite
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x0E;
    dbbus_tx_data[3] = 0x02;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );


    // set pin high
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x10;
    dbbus_tx_data[3] = 0x08;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    dbbusDWIICIICNotUseBus();
    dbbusDWIICNotStopMCU();
    dbbusDWIICExitSerialDebugMode();


    drvISP_EntryIspMode();
    drvISP_ChipErase();
    _HalTscrHWReset();
    mdelay ( 300 );

    // 2.Program and Verify
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();


    // Disable the Watchdog
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x60;
    dbbus_tx_data[3] = 0x55;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x61;
    dbbus_tx_data[3] = 0xAA;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // Stop MCU
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x0F;
    dbbus_tx_data[2] = 0xE6;
    dbbus_tx_data[3] = 0x01;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );


    // set FRO to 50M
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x11;
    dbbus_tx_data[2] = 0xE2;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
    dbbus_rx_data[0] = 0;
    dbbus_rx_data[1] = 0;
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
    TP_DEBUG ( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
    dbbus_tx_data[3] = dbbus_rx_data[0] & 0xF7;  //Clear Bit 3
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );


    // set MCU clock,SPI clock =FRO
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x22;
    dbbus_tx_data[3] = 0x00;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x23;
    dbbus_tx_data[3] = 0x00;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // Enable slave's ISP ECO mode
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x08;
    dbbus_tx_data[2] = 0x0c;
    dbbus_tx_data[3] = 0x08;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // Enable SPI Pad
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x02;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
    TP_DEBUG ( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
    dbbus_tx_data[3] = ( dbbus_rx_data[0] | 0x20 ); //Set Bit 5
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );


    // WP overwrite
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x0E;
    dbbus_tx_data[3] = 0x02;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );


    // set pin high
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x10;
    dbbus_tx_data[3] = 0x08;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    dbbusDWIICIICNotUseBus();
    dbbusDWIICNotStopMCU();
    dbbusDWIICExitSerialDebugMode();

    ///////////////////////////////////////
    // Start to load firmware
    ///////////////////////////////////////
    drvISP_EntryIspMode();

    for ( i = 0; i < 94; i++ ) // total  94 KB : 1 byte per R/W
    {
        drvISP_Program ( i, temp[i] ); // program to slave's flash
        drvISP_Verify ( i, temp[i] ); //verify data
    }
    TP_DEBUG ( "update OK\n" );
    drvISP_ExitIspMode();
    FwDataCnt = 0;
    
    return size;
}
#endif

static DEVICE_ATTR ( update, 0777, firmware_update_show, firmware_update_store );

/*test=================*/
static ssize_t firmware_clear_show ( struct device *dev,
                                     struct device_attribute *attr, char *buf )
{
    u16 k = 0, i = 0, j = 0;
    u8 bWriteData[5] =
    {
        0x10, 0x03, 0, 0, 0
    };
    u8 RX_data[256];
    u8 bWriteData1 = 0x12;
    u32 addr = 0;
    u32 timeOutCount = 0;
    for ( k = 0; k < 94; i++ ) // total  94 KB : 1 byte per R/W
    {
        addr = k * 1024;
        for ( j = 0; j < 8; j++ ) //128*8 cycle
        {
            bWriteData[2] = ( u8 ) ( ( addr + j * 128 ) >> 16 );
            bWriteData[3] = ( u8 ) ( ( addr + j * 128 ) >> 8 );
            bWriteData[4] = ( u8 ) ( addr + j * 128 );
//            msctpc_LoopDelay ( 1 );        // delay about 100us*****
        udelay(100);

            timeOutCount = 0;
            while ( ( drvISP_ReadStatus() & 0x01 ) == 0x01 )
            {
                timeOutCount++;
                if ( timeOutCount >= 100000 ) break; /* around 1 sec timeout */
            }


            HalTscrCDevWriteI2CSeq ( FW_UPDATE_ADDR_MSG21XX, bWriteData, 5 ); //write read flash addr
//            msctpc_LoopDelay ( 1 );        // delay about 100us*****
        udelay(100);
            drvISP_Read ( 128, RX_data );
            HalTscrCDevWriteI2CSeq ( FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1 ); //cmd end
            for ( i = 0; i < 128; i++ ) //log out if verify error
            {
                if ( RX_data[i] != 0xFF )
                {
                    TP_DEBUG ( "k=%d,j=%d,i=%d===============erase not clean================", k, j, i );
                }
            }
        }
    }
    TP_DEBUG ( "read finish\n" );
    return sprintf ( buf, "%s\n", fw_version );
}

static ssize_t firmware_clear_store ( struct device *dev,
                                      struct device_attribute *attr, const char *buf, size_t size )
{

    u8 dbbus_tx_data[4];
    unsigned char dbbus_rx_data[2] = {0};

    _HalTscrHWReset();
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();


    // Disable the Watchdog
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x60;
    dbbus_tx_data[3] = 0x55;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x61;
    dbbus_tx_data[3] = 0xAA;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // Stop MCU
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x0F;
    dbbus_tx_data[2] = 0xE6;
    dbbus_tx_data[3] = 0x01;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // set FRO to 50M
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x11;
    dbbus_tx_data[2] = 0xE2;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
    dbbus_rx_data[0] = 0;
    dbbus_rx_data[1] = 0;
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
    TP_DEBUG ( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
    dbbus_tx_data[3] = dbbus_rx_data[0] & 0xF7;  //Clear Bit 3
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // set MCU clock,SPI clock =FRO
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x22;
    dbbus_tx_data[3] = 0x00;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x23;
    dbbus_tx_data[3] = 0x00;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // Enable slave's ISP ECO mode
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x08;
    dbbus_tx_data[2] = 0x0c;
    dbbus_tx_data[3] = 0x08;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );
    
    // Enable SPI Pad
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x02;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
    TP_DEBUG ( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
    dbbus_tx_data[3] = ( dbbus_rx_data[0] | 0x20 ); //Set Bit 5
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    //WP overwrite
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x0E;
    dbbus_tx_data[3] = 0x02;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    //set pin high
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x10;
    dbbus_tx_data[3] = 0x08;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    dbbusDWIICIICNotUseBus();
    dbbusDWIICNotStopMCU();
    dbbusDWIICExitSerialDebugMode();


    ///////////////////////////////////////
    // Start to load firmware
    ///////////////////////////////////////
    drvISP_EntryIspMode();
    TP_DEBUG ( "chip erase+\n" );
    drvISP_ChipErase();
    TP_DEBUG ( "chip erase-\n" );
    drvISP_ExitIspMode();
    
    return size;
}

static DEVICE_ATTR ( clear, 0777, firmware_clear_show, firmware_clear_store );

/*test=================*/
/*Add by Tracy.Lin for update touch panel firmware and get fw version*/

static ssize_t firmware_version_show ( struct device *dev,
                                       struct device_attribute *attr, char *buf )
{
    TP_DEBUG ( "*** firmware_version_show fw_version = %s***\n", fw_version );
    return sprintf ( buf, "%s\n", fw_version );
}

static ssize_t firmware_version_store ( struct device *dev,
                                        struct device_attribute *attr, const char *buf, size_t size )
{
    unsigned char dbbus_tx_data[3];
    unsigned char dbbus_rx_data[4] ;
    unsigned short major = 0, minor = 0;

    fw_version = kzalloc ( sizeof ( char ), GFP_KERNEL );
    
    // SM-BUS GET FW VERSION
    dbbus_tx_data[0] = 0x53;
    dbbus_tx_data[1] = 0x00;
    dbbus_tx_data[2] = 0x74;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX_TP, &dbbus_tx_data[0], 3 );
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX_TP, &dbbus_rx_data[0], 4 );

    major = ( dbbus_rx_data[1] << 8 ) + dbbus_rx_data[0];
    minor = ( dbbus_rx_data[3] << 8 ) + dbbus_rx_data[2];

    TP_DEBUG ( "***major = %d ***\n", major );
    TP_DEBUG ( "***minor = %d ***\n", minor );
    sprintf ( fw_version, "%03d%03d", major, minor );
    TP_DEBUG ( "***fw_version = %s ***\n", fw_version );


    return size;
}
static DEVICE_ATTR ( version, 0777, firmware_version_show, firmware_version_store );

static ssize_t firmware_data_show ( struct device *dev,
                                    struct device_attribute *attr, char *buf )
{
    return FwDataCnt;
}

static ssize_t firmware_data_store ( struct device *dev,
                                     struct device_attribute *attr, const char *buf, size_t size )
{
    int i;
    TP_DEBUG ( "***FwDataCnt = %d ***\n", FwDataCnt );
    memcpy ( temp[FwDataCnt], buf, 1024 );
    FwDataCnt++;
    return size;
}
static DEVICE_ATTR ( data, 0777, firmware_data_show, firmware_data_store );
#endif
//---------------------------------------------------------------------------
static void msg21xx_suspend ( struct early_suspend *h )
{
    //  gpio_set_value(MSG21XX_RESET_GPIO, 0);

	disable_irq(msg21xx_irq);

	gpio_set_value ( MSG21XX_RESET_GPIO, 0 );
	
	tegra_gpio_enable(TEGRA_GPIO_PT5);
	gpio_request(TEGRA_GPIO_PT5, "ft5x0x-i2c-scl-sleep");
	gpio_direction_output(TEGRA_GPIO_PT5, 0);
	gpio_set_value(TEGRA_GPIO_PT5, 0);

	tegra_gpio_enable(TEGRA_GPIO_PT6);
	gpio_request(TEGRA_GPIO_PT6, "ft5x0x-i2c-sda-sleep");
	gpio_direction_output(TEGRA_GPIO_PT6, 0);
	gpio_set_value(TEGRA_GPIO_PT6, 0);
	
}

static void msg21xx_resume ( struct early_suspend *h )
{
    //  gpio_set_value(MSG21XX_RESET_GPIO, 1);
    	tegra_gpio_disable(TEGRA_GPIO_PT5);
	tegra_gpio_disable(TEGRA_GPIO_PT6);

	gpio_set_value ( MSG21XX_RESET_GPIO, 1 );
	gpio_set_value ( MSG21XX_RESET_GPIO, 0 );
	mdelay ( 200 ); // Note that the RST must be in LOW 10ms at least
	gpio_set_value ( MSG21XX_RESET_GPIO, 1 );
	// Enable the interrupt service thread/routine for INT after 50ms
	mdelay ( 50 );

	enable_irq(msg21xx_irq);
	
}

static void msg21xx_chip_init ( void )
{
    // After the LCD is on, power on the TP controller
    gpio_direction_output ( MSG21XX_RESET_GPIO, 1 );
    gpio_set_value ( MSG21XX_RESET_GPIO, 1 );
    gpio_set_value ( MSG21XX_RESET_GPIO, 0 );
    mdelay ( 200 ); // Note that the RST must be in LOW 10ms at least
    gpio_set_value ( MSG21XX_RESET_GPIO, 1 );
    // Enable the interrupt service thread/routine for INT after 50ms
    mdelay ( 50 );
}

static u8 Calculate_8BitsChecksum ( u8 *msg, s32 s32Length )
{
    s32 s32Checksum = 0;
    s32 i;

    for ( i = 0 ; i < s32Length; i++ )
    {
        s32Checksum += msg[i];
    }

    return ( u8 ) ( ( -s32Checksum ) & 0xFF );
}


static void msg21xx_data_disposal ( struct work_struct *work )
{
    u8 val[8] = {0};
    u8 Checksum = 0;
    u8 i;
    u32 delta_x = 0, delta_y = 0;
    u32 u32X = 0;
    u32 u32Y = 0;
    u8 touchkeycode = 0;
    TouchScreenInfo_t touchData;
    static u32 preKeyStatus = 0;
    //static u32 preFingerNum=0;

#define SWAP_X_Y   (1)
// #define REVERSE_X  (1)
#define REVERSE_Y  (1)
#ifdef SWAP_X_Y
    int tempx;
    int tempy;
#endif

    //_TRACE((TSCR_LEVEL_C_TYP, "DrvTscrCGetData in"));
    i2c_master_recv ( msg21xx_i2c_client, &val[0], REPORT_PACKET_LENGTH );
    Checksum = Calculate_8BitsChecksum ( &val[0], 7 ); //calculate checksum

    if ( ( Checksum == val[7] ) && ( val[0] == 0x52 ) ) //check the checksum  of packet
    {
        u32X = ( ( ( val[1] & 0xF0 ) << 4 ) | val[2] );   //parse the packet to coordinates
        u32Y = ( ( ( val[1] & 0x0F ) << 8 ) | val[3] );

        delta_x = ( ( ( val[4] & 0xF0 ) << 4 ) | val[5] );
        delta_y = ( ( ( val[4] & 0x0F ) << 8 ) | val[6] );

#ifdef SWAP_X_Y
        tempy = u32X;
        tempx = u32Y;
        u32X = tempx;
        u32Y = tempy;

        tempy = delta_x;
        tempx = delta_y;
        delta_x = tempx;
        delta_y = tempy;
#endif

#ifdef REVERSE_X
        u32X = 2047 - u32X;
        delta_x = 4095 - delta_x;
#endif

#ifdef REVERSE_Y
        u32Y = 2047 - u32Y;
        delta_y = 4095 - delta_y;
#endif
        //DBG("[HAL] u32X = %x, u32Y = %x", u32X, u32Y);
        //DBG("[HAL] delta_x = %x, delta_y = %x", delta_x, delta_y);

        if ( ( val[1] == 0xFF ) && ( val[2] == 0xFF ) && ( val[3] == 0xFF ) && ( val[4] == 0xFF ) && ( val[6] == 0xFF ) )
        {
            touchData.Point[0].X = 0; // final X coordinate
            touchData.Point[0].Y = 0; // final Y coordinate

            if ( ( val[5] == 0x0 ) || ( val[5] == 0xFF ) )
            {
                touchData.nFingerNum = 0; //touch end
                touchData.nTouchKeyCode = 0; //TouchKeyMode
                touchData.nTouchKeyMode = 0; //TouchKeyMode
            }
            else
            {
                touchData.nTouchKeyMode = 1; //TouchKeyMode
                touchData.nTouchKeyCode = val[5]; //TouchKeyCode
                touchData.nFingerNum = 1;
            }
        }
        else
        {
            touchData.nTouchKeyMode = 0; //Touch on screen...

            if (
#ifdef REVERSE_X
                ( delta_x == 4095 )
#else
                ( delta_x == 0 )
#endif
                &&
#ifdef REVERSE_Y
                ( delta_y == 4095 )
#else
                ( delta_y == 0 )
#endif
            )
            {
                touchData.nFingerNum = 1; //one touch
                touchData.Point[0].X = ( u32X * MS_TS_MSG21XX_X_MAX ) / 2048;
                touchData.Point[0].Y = ( u32Y * MS_TS_MSG21XX_Y_MAX ) / 2048;
            }
            else
            {
                u32 x2, y2;

                touchData.nFingerNum = 2; //two touch

                // Finger 1
                touchData.Point[0].X = ( u32X * MS_TS_MSG21XX_X_MAX ) / 2048;
                touchData.Point[0].Y = ( u32Y * MS_TS_MSG21XX_Y_MAX ) / 2048;

                // Finger 2
                if ( delta_x > 2048 )   //transform the unsigh value to sign value
                {
                    delta_x -= 4096;
                }
                if ( delta_y > 2048 )
                {
                    delta_y -= 4096;
                }

                x2 = ( u32 ) ( u32X + delta_x );
                y2 = ( u32 ) ( u32Y + delta_y );

                touchData.Point[1].X = ( x2 * MS_TS_MSG21XX_X_MAX ) / 2048;
                touchData.Point[1].Y = ( y2 * MS_TS_MSG21XX_Y_MAX ) / 2048;
            }
        }

        // report...
        if ( touchData.nTouchKeyMode )
        {
/*
            if ( touchData.nTouchKeyCode == 1 )
                touchkeycode = KEY_HOME;
            if ( touchData.nTouchKeyCode == 2 )
                touchkeycode = KEY_MENU;
            if ( touchData.nTouchKeyCode == 4 )
                touchkeycode = KEY_BACK;
            if ( touchData.nTouchKeyCode == 8 )
                touchkeycode = KEY_SEARCH;

            if ( preKeyStatus != touchkeycode )
            {
                preKeyStatus = touchkeycode;
                input_report_key ( input, touchkeycode, 1 );
                //printk("&&&&&&&&useful key code report touch key code = %d\n",touchkeycode);
            }
            input_sync ( input );
*/
			
        }
        else
        {
            preKeyStatus = 0; //clear key status..

            if ( ( touchData.nFingerNum ) == 0 ) //touch end
            {
                //preFingerNum=0;
/*
                input_report_key ( tpd->dev, KEY_MENU, 0 );
                input_report_key ( tpd->dev, KEY_HOME, 0 );
                input_report_key ( tpd->dev, KEY_BACK, 0 );
                input_report_key ( tpd->dev, KEY_SEARCH, 0 );
*/
                input_report_abs ( input, ABS_MT_TOUCH_MAJOR, 0 );
                input_mt_sync ( input );
                input_sync ( input );

            }
            else //touch on screen
            {
                /*
                if(preFingerNum!=touchData.nFingerNum)   //for one touch <--> two touch issue
                {
                    printk("langwenlong number has changed\n");
                    preFingerNum=touchData.nFingerNum;
                    input_report_abs(input, ABS_MT_TOUCH_MAJOR, 0);
                    input_mt_sync(input);
                    input_sync(input);
                }
                */

                for ( i = 0; i < ( touchData.nFingerNum ); i++ )
                {
                    input_report_abs ( input, ABS_MT_PRESSURE, 255 );
                    input_report_abs ( input, ABS_MT_TOUCH_MAJOR, 255 );
                    input_report_abs ( input, ABS_MT_POSITION_X, touchData.Point[i].X );
                    input_report_abs ( input, ABS_MT_POSITION_Y, touchData.Point[i].Y );
                    input_mt_sync ( input );

//			printk("&&&&&&&[i = %d], POSITION_X= %d, POSITION_y = %d\n", i, touchData.Point[i].X, touchData.Point[i].Y);

                }

                input_sync ( input );
            }
        }
    }
    else
    {
        //DBG("Packet error 0x%x, 0x%x, 0x%x", val[0], val[1], val[2]);
        //DBG("             0x%x, 0x%x, 0x%x", val[3], val[4], val[5]);
        //DBG("             0x%x, 0x%x, 0x%x", val[6], val[7], Checksum);
        //printk ( KERN_ERR "err status in tp\n" );
    }

    enable_irq ( msg21xx_irq );
}


static int msg21xx_ts_open ( struct input_dev *dev )
{
    return 0;
}

static void msg21xx_ts_close ( struct input_dev *dev )
{
    printk ( "msg21xx_ts_close\n" );
}


static int msg21xx_init_input ( void )
{
    int err;

    input = input_allocate_device();
    input->name = msg21xx_i2c_client->name;
    input->phys = "I2C";
    input->id.bustype = BUS_I2C;
    input->dev.parent = &msg21xx_i2c_client->dev;
    input->open = msg21xx_ts_open;
    input->close = msg21xx_ts_close;

    set_bit ( EV_ABS, input->evbit );
    set_bit ( EV_SYN, input->evbit );
    set_bit ( EV_KEY, input->evbit );
/*
    set_bit ( TOUCH_KEY_BACK, input->keybit );
    set_bit ( TOUCH_KEY_MENU, input->keybit );
    set_bit ( TOUCH_KEY_HOME, input->keybit );
*/

//    input_set_abs_params ( input, ABS_PRESSURE, 0, 255, 0, 0 );
    input_set_abs_params ( input, ABS_MT_PRESSURE, 0, 255, 0, 0 );
    input_set_abs_params ( input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0 );
    input_set_abs_params ( input, ABS_MT_POSITION_X, 0, MS_TS_MSG21XX_X_MAX, 0, 0 );
    input_set_abs_params ( input, ABS_MT_POSITION_Y, 0, MS_TS_MSG21XX_Y_MAX, 0, 0 );

    err = input_register_device ( input );
    if ( err )
        goto fail_alloc_input;

fail_alloc_input:
    return 0;
}
static irqreturn_t msg21xx_interrupt ( int irq, void *dev_id )
{
    disable_irq_nosync ( msg21xx_irq );
    schedule_work ( &msg21xx_wq );
    return IRQ_HANDLED;
}


static int __devinit msg21xx_probe ( struct i2c_client *client, const struct i2c_device_id *id )
{
    int  err = 0;

    msg21xx_i2c_client = client;
    this_client = client;

    gpio_request ( MSG21XX_INT_GPIO, "interrupt" );
    gpio_request(MSG21XX_RESET_GPIO, "reset");
    gpio_direction_input ( MSG21XX_INT_GPIO );
//    gpio_set_value(MSG21XX_INT_GPIO, 1);

    msg21xx_chip_init();
    INIT_WORK ( &msg21xx_wq, msg21xx_data_disposal );
    msg21xx_init_input();
//    msg21xx_irq = IRQ_GPIO ( MSG21XX_INT_GPIO );
    msg21xx_irq = TEGRA_GPIO_TO_IRQ(MSG21XX_INT_GPIO);
    err = request_irq ( msg21xx_irq, msg21xx_interrupt,
                        /*IRQF_TRIGGER_FALLING*/IRQF_TRIGGER_RISING, "msg21xx", NULL );
    if ( err != 0 )
    {
        printk ( "%s: cannot register irq\n", __func__ );
        goto exit;
    }

    early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
    early_suspend.suspend = msg21xx_suspend;
    early_suspend.resume = msg21xx_resume;
    register_early_suspend ( &early_suspend );

#ifdef MSG2133_SOFTWARE_UPDATE
	msg2133_init_fw_class();
#endif

/* 20130220, JimmySu add read msg21xx firmware version*/
/*fw version*/
	err = sysfs_create_group(&client->dev.kobj, &msg21xx_attr_group);
	if (err)
		printk("[Touch](%s)failed to create flash sysfs files\n",__FUNCTION__);
/* 20130220, JimmySu add read msg21xx firmware version*/

#ifdef MSG2133_AUTO_UPDATE
//	msg2133_auto_update_fw();
//	firmware_update_store();
#endif	

#ifdef MSG2133_DRIVER_UPDATE
	INIT_DELAYED_WORK(&delay_work_driver_fw_check, msg2133_fw_check);

	schedule_delayed_work(&delay_work_driver_fw_check, HZ);	
#endif	

	touch_board_id = nabijr_get_board_strap();

    // frameware upgrade
#ifdef __FIRMWARE_UPDATE__
    firmware_class = class_create ( THIS_MODULE, "ms-touchscreen-msg20xx" );
    if ( IS_ERR ( firmware_class ) )
        pr_err ( "Failed to create class(firmware)!\n" );
    firmware_cmd_dev = device_create ( firmware_class,
                                       NULL, 0, NULL, "device" );
    if ( IS_ERR ( firmware_cmd_dev ) )
        pr_err ( "Failed to create device(firmware_cmd_dev)!\n" );

    // version
    if ( device_create_file ( firmware_cmd_dev, &dev_attr_version ) < 0 )
        pr_err ( "Failed to create device file(%s)!\n", dev_attr_version.attr.name );
    // update
    if ( device_create_file ( firmware_cmd_dev, &dev_attr_update ) < 0 )
        pr_err ( "Failed to create device file(%s)!\n", dev_attr_update.attr.name );
    // data
    if ( device_create_file ( firmware_cmd_dev, &dev_attr_data ) < 0 )
        pr_err ( "Failed to create device file(%s)!\n", dev_attr_data.attr.name );
    // clear
    if ( device_create_file ( firmware_cmd_dev, &dev_attr_clear ) < 0 )
        pr_err ( "Failed to create device file(%s)!\n", dev_attr_clear.attr.name );

    dev_set_drvdata ( firmware_cmd_dev, NULL );
#endif

    return 0;

exit:
    return err;
}

static int __devexit msg21xx_remove ( struct i2c_client *client )
{
    return 0;
}


static const struct i2c_device_id msg21xx_id[] =
{
    { "ms-msg21xx", 0x26 },
    { }
};

MODULE_DEVICE_TABLE ( i2c, msg21xx_id );


static struct i2c_driver msg21xx_driver =
{
    .driver = {
        .name = "ms-msg21xx",
        .owner = THIS_MODULE,
    },
    .probe = msg21xx_probe,
    .remove = __devexit_p ( msg21xx_remove ),
    .id_table = msg21xx_id,
};


static int __init msg21xx_init ( void )
{
    int err;
    err = i2c_add_driver ( &msg21xx_driver );
    if ( err )
    {
        printk ( KERN_WARNING "msg21xx  driver failed "
                 "(errno = %d)\n", err );
    }
    else
    {
        printk ( "Successfully added driver %s\n",
                 msg21xx_driver.driver.name );
    }
    return err;
}

static void __exit msg21xx_cleanup ( void )
{
    i2c_del_driver ( &msg21xx_driver );
}


module_init ( msg21xx_init );
module_exit ( msg21xx_cleanup );

MODULE_AUTHOR ( "Mstar semiconductor" );
MODULE_DESCRIPTION ( "Driver for msg21xx Touchscreen Controller" );
MODULE_LICENSE ( "GPL" );

