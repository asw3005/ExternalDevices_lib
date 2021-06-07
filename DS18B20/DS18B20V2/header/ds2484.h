/*
 *	@brief DS2484 driver.
 *	Created 03.08.21
 *	DS2484_H_
 *
 **/

#ifndef DS2484_H_				  
#define DS2484_H_

#include "stm32f1xx_hal.h"

#ifndef NULL
#define NULL (void *)0	
#endif //NULL

/**
 ** @brief Device address
 **/
#define DS2484_ADDR						0x18
#define DS2484_ADDR_SHIFTED             0x30     

#define DS2484_ErrTimeout				0x01
#define DS2484_NoErrTimeout				0x00
/*
 * @brief Valid read pointer codes.
 *
 **/
typedef enum 
{
	DS2484_CFG_PREG       = 0xC3,
	DS2484_STATUS_PREG    = 0xF0,
	DS2484_READ_DATA_PREG = 0xE1,
	DS2484_PORT_CFG_PREG  = 0xB4
	
} DS2484_PReadCode;
	
/*
 * @brief I2C communication command.
 *
 **/
typedef enum
{
	DS2484_DEVICE_RESET					= 0xF0,
	DS2484_SET_READ_POINTER				= 0xE1,
	DS2484_WRITE_DEVICE_CONFIGURATION	= 0xD2,
	DS2484_1W_PORT_ADJUST				= 0xC3,
	DS2484_1W_RESET						= 0xB4,
	DS2484_1W_SINGLE_BIT				= 0x87,
	DS2484_1W_WRITE_BYTE				= 0xA5,
	DS2484_1W_READ_BYTE					= 0x96,
	DS2484_1W_TRIPLET					= 0x78
	
} DS2484_Command;

/*
 *	@brief Delay function typedef pointer. 
 *	
 *	@param period : Time in milliseconds.
 *
 **/
typedef void(*delay_fptr)(uint32_t period);

/*
 *	@brief Tx, Rx function typedef pointer. 
 *	
 *	@param *buffer : Buffer for transmit or receive data.
 *	@param size : Amount bytes of data.
 *
 **/
typedef int8_t(*ds2484_rxtx_fptr)(uint8_t address, uint8_t *pdata, uint16_t size);

/*
 * @brief Device configuration register bit assigment.
 *
 **/
typedef	union
	{
		uint8_t CFGReg;
		struct {
			uint8_t LOWER_NIBBLE : 4;
			uint8_t UPPER_NIBBLE : 4;
		};
		
		struct
		{
			//The APU bit controls whether an active pullup (lowimpedance transistor) or a passive pullup (RWPU resistor)
			//is used to drive a 1-Wire line from low to high. When APU = 0, active pullup is disabled (resistor mode).
			//Enabling active pullup is generally recommended for best 1-Wire bus performance. The active pullup does
			//not apply to the rising edge of a recovery after a short on the 1-Wire line.
			uint8_t APU				: 1;
			//The PDN bit is used to remove power from the 1-Wire port. While PDN is 1, no 1 - Wire communication is possible.
			//To end the 1 - Wire power - down state, the PDN bit must be changed to 0. Writing both the PDN bit and the SPU
			//bit to 1 forces the SPU bit to 0.
			uint8_t PDN				: 1;
			//The SPU bit is used to activate the strong pullup function prior to a 1 - Wire Write Byte or 1 - Wire Single Bit
			//command. If SPU is 1 and APU is 0, the DS2484 treats the rising edge of the time slot as if the active pullup was 
			//activated, but uses VIH1 as the threshold to enable the strong pullup.If SPU is 1 and APU is 1, the threshold 
			//voltage to enable the strong pullup is VIAPO.Once enabled, in contrast to the active pullup, the internal pullup 
			//transistor remains conducting, as shown in Figure 3, until one of three events occurs : the DS2484 receives a 
			//command that generates 1 - Wire communication(the typical case), the SPU bit in the Device Configuration register
			//is written to 0, or the DS2484 receives the Device Reset command.
			uint8_t SPU				: 1;
			//The 1WS bit determines the timing of any 1 - Wire communication generated by the DS2484. All 1 - Wire slave devices
			//support standard speed(1WS = 0). Many 1 - Wire devices can also communicate at a higher data rate, called overdrive 
			//speed. To change from standard to overdrive speed, a 1 - Wire device needs to receive an Overdrive - Skip ROM or 
			//Overdrive - Match ROM command, as explained in the Maxim 1 - Wire IC data sheets. Device Configuration register 
			//with the 1WS bit as 0, followed by a 1 - Wire Reset command, changes the DS2484 and any 1 - Wire devices on the 
			//active 1 - Wire line back to standard speed.
			uint8_t ONEWS			: 1;
			//When writing to the Device Configuration register, the new data is accepted only if the upper nibble
			//(bits 7 to 4) is the one’s complement of the lower nibble (bits 3 to 0).When read, the upper nibble 
			//is always 0h.
			uint8_t ONES_COMPLEMENT : 4;		
		};
		
} DS2484_CFGReg_t;

/*
 * @brief Device status register bit assigment.
 *
 **/
typedef union 
{
	uint8_t StatusReg;
	struct
	{
		//The 1WB bit reports to the host processor whether the 1 - Wire line is busy. During 1 - Wire communication 1WB
		//is 1; once the command is completed, 1WB returns to its default 0.
		uint8_t ONEWB	: 1;
		//The PPD bit is updated with every 1-Wire Reset command. If the DS2484 detects a logic 0 on the 1 - Wire line
		//at tMSP during the presence - detect cycle, the PPD bit is set to 1. This bit returns to its default 0 if 
		//there is no presence pulse during a subsequent 1 - Wire Reset command.
		uint8_t PPD		: 1;
		//The SD bit is updated with every 1-Wire Reset command. If the DS2484 detects a logic 0 on the 1 - Wire line at
		//tSI during the presence - detect cycle, the SD bit is set to 1. This bit returns to its default 0 with a 
		//subsequent 1 - Wire Reset command, provided that the short has been removed. If the 1 - Wire line is shorted at 
		//tMSP, the PPD bit is also set.
		uint8_t SD		: 1;
		//The LL bit reports the logic state of the active 1 - Wire line without initiating any 1 - Wire communication.
		//The 1 - Wire line is sampled for this purpose every time the Status register is read.
		uint8_t  LL		: 1;
		//If the RST bit is 1, the DS2484 has performed an internal reset cycle, either caused by a power - on reset or 
		//from executing the Device Reset command. The RST bit is cleared automatically when the DS2484 executes a Write
		//Device Configuration command to restore the selection of the desired 1 - Wire features.
		uint8_t RST		: 1;
		//The SBR bit reports the logic state of the active 1-Wire line sampled at tMSR of a 1 - Wire Single Bit command 
		//or the first bit of a 1 - Wire Triplet command.The power - on default of SBR is 0. If the 1 - Wire Single Bit 
		//command sends a 0 bit, SBR should be 0. With a 1 - Wire Triplet command, SBR could be 0 as well as 1, depending 
		//on the response of the 1 - Wire devices connected. The same result applies to a 1 - Wire Single Bit command that 
		//sends a 1 bit.
		uint8_t SBR		: 1;
		//The TSB bit reports the logic state of the active 1-Wire line sampled at tMSR of the second bit of a 1 - Wire 
		//Triplet command. The power - on default of TSB is 0. This bit is updated only with a 1 - Wire Triplet command and 
		//has no function with other commands.
		uint8_t  TSB	: 1;
		//Whenever a 1-Wire Triplet command is executed, this bit reports to the host processor the search direction that
		//was chosen by the third bit of the triplet. The power - on default of DIR is 0. This bit is updated only with a 
		//1 - Wire Triplet command and has no function with other commands.
		uint8_t DIR		: 1;		
	};
		
} DS2484_StatusReg_t;

/*
 * @brief Device port configuration register bit assigment.
 *
 **/
typedef union 
{
	uint8_t PortCFGReg;
	struct
	{
		//The Port Configuration register allows verifying the settings for the 1 - Wire port(Table 4).The Adjust 1 - Wire
		//Port command positions the read pointer to the Port Configuration register for the host processor to read with minimal 
		//protocol overhead.
		//When reading the Port Configuration register, the parameter values are reported in this sequence :
		//	Parameter 000(tRSTL) standard speed, overdrive speed,
		//	Parameter 001(tMSP) standard speed, overdrive speed,
		//	Parameter 010(tW0L) standard speed, overdrive speed,
		//	Parameter 011(tREC0),
		//	Parameter 100(RWPU).
		//If one continues reading, the parameter number rolls over to 000 and one receives the same data again.
		uint8_t VAL_3_0			: 4;
		//Upper 4 bits read from the port configuration register are always 0.
		uint8_t RESERVED_7_4	: 4;	
	};
		
} DS2484_PortCFGReg_t;

/*
 * @brief Device control byte type.
 * 
 *PARAMETER		PARAMETER 000	PARAMETER 001	PARAMETER 010	PARAMETER 011	PARAMETER 100
 *	VALUE		  tRSTL (μs)      tMSP (μs)		  tW0L (μs)		  tREC0 (μs)       RWPU (W)
 *	CODE		OD = 0 OD = 1	OD = 0 OD = 1	OD = 0 OD = 1	   OD = N/A	      OD = N/A
 *	0000		440		 44		  58	5.5		   52	5.0			 2.75			 500
 *	0001		460		 46		  58	5.5		   54	5.5			 2.75			 500
 *	0010		480		 48		  60	6.0		   56	6.0			 2.75			 500
 *	0011		500		 50		  62	6.5		   58	6.5			 2.75			 500
 *	0100		520		 52		  64	7.0		   60	7.0			 2.75			 500
 *	0101		540		 54		  66	7.5		   62	7.5			 2.75			 500
 *	0110		560		 56		  68	8.0		   64	8.0			 5.25			 1000        default
 *	0111		580		 58		  70	8.5		   66	8.5			 7.75			 1000
 *	1000		600		 60		  72	9.0		   68	9.0			 10.25			 1000
 *	1001		620		 62		  74	9.5		   70	9.5			 12.75			 1000
 *	1010		640		 64		  76	10.0	   70	10			 15.25			 1000
 *	1011		660		 66		  76	10.5	   70	10			 17.75			 1000
 *	1100		680		 68		  76	11.0	   70	10			 20.25			 1000
 *	1101		700		 70		  76	11.0	   70	10			 22.75			 1000
 *	1110		720		 72		  76	11.0	   70	10			 25.25			 1000
 *	1111		740		 74		  76	11.0	   70	10			 25.25			 1000
 *
 **/
typedef union 
{
	uint8_t ControlByte;
	struct
	{
		//Parameter value code
		uint8_t VAL_3_0		: 4;
		//Overdrive control.
		//0 : the value provided applies to the standard speed setting.
		//1 : the value provided applies to the overdrive speed setting.
		uint8_t OD_4		: 1;
		//Parameter 000 tRSTL.
		//Parameter 001 tMSP.
		//Parameter 010 tW0L.
		//Parameter 011 tREC0, the OD flag does not apply (don’t care).
		//Parameter 100 RWPU, the OD flag does not apply (don’t care).
		uint8_t P_7_5		: 3;	
	};
		
} DS2484_ControlByte_t;

/*
 * @brief Tx/Rx data struct.
 *
 **/
typedef struct
{
	uint8_t Command;
	uint8_t TxDataByte[2];
	uint8_t RxDataByte[2];
	
} DS2484_DataCommunication_t;

/*
 *	@brief General data struct instance.
 *
 **/
typedef struct
{
	//Pointer to the counter of left data.
	uint16_t* isReceiveComplete;
	//
	DS2484_DataCommunication_t data_struct;
	//Internal registers, status and configuration.
	//DS2484_InternalReg internal_reg;
	//Pointers for the rx, tx delay functions.
	delay_fptr delay;
	ds2484_rxtx_fptr i2c_tx_data;
	ds2484_rxtx_fptr i2c_rx_data;
	
} DS2484_GInst_t;

/* Public function prototypes. */

/* 1-Wire bus functions. */
uint8_t DS2484_1WireReset(DS2484_GInst_t *device);
void DS2484_1WireSPU(DS2484_GInst_t *device, uint8_t state);
uint8_t DS2484_1WireSingleBit(DS2484_GInst_t *device, uint8_t time_slot_value);
uint8_t DS2484_1WireWriteByte(DS2484_GInst_t *device, uint8_t byte);
uint8_t DS2484_1WireReadByte(DS2484_GInst_t *device);
uint8_t DS2484_1WireWriteData(DS2484_GInst_t *device, uint8_t* data, uint8_t size);
void DS2484_1WireReadData(DS2484_GInst_t *device, uint8_t* data, uint8_t size);

/* Device functions. */
void DS2484_DeviceReset(DS2484_GInst_t *device);
void DS2484_WriteDeviceConfiguration(DS2484_GInst_t *device, uint8_t apu, uint8_t pdn, uint8_t spu, uint8_t onews);
void DS2484_1WirePortAdjust(DS2484_GInst_t *device, uint8_t val, uint8_t od, uint8_t p);


#endif /* DS2484_H_ */