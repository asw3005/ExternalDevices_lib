/*
 * @brief Common.h
 *
 * Created on: Jul 16, 2023
 * Author: asw3005
 *
 */

#ifndef COMMON_H_
#define COMMON_H_

#include "stm32f4xx.h"

/* EEP data and constants. */

/* Page's addresses. */
#define EEP_DATA_ADDR			0x50
#define	EEP_DATA_ADDR_SHIFTED	0xA0
#define EEP_ID_PAGE				0x58
#define EEP_ID_PAGE_SHIFTED		0xB0

/*
	@brief EEP action constants.
*/
typedef enum {

	EEP_WRITE_TO,
	EEP_READ_FROM

} EXT_MEM_RW_t;

/*
 * @brief Page's codes.
 */
typedef enum
{
	ST_MANUFACTURER_CODE	= 0x00,
	I2C_FAMILY_CODE			= 0x01,
	MEMORY_DENSITY_CODE		= 0x02,
	M24128_ERT				= 0x04

} M24128_ID_PAGE_ADDR_t;

/*
 * @brief Internal memory organization struct.
 */
typedef	struct /*__attribute__((aligned(1), packed))*/ {
	/* Memory address to select page and byte on the page. */
	union {
		uint16_t MemAddr;
		struct {
			uint8_t BYTE_ADDR;
			uint8_t PAGE_ADDR;
		};
	};

} M24128_MEM_DATA_t;

/* */
#define MPCB_DEFAULT_ADDRESS		0
#define MPCB_EEP_CLEAR_ADDR			255
#define MAX_MPCB_NUMBER				3
#define MAX_UPCB_NUMBER				6
#define MAX_SGU_CHANNELS			4
#define NO_DEVICE_ON_PCB			-50
#define LED_SCREEN_TIMEOUT			10000
#define DEFAULT_CONNECTION_TIMEOUT	30000

/*
	@brief Device's command sets throw the UART.
*/
typedef enum {

	/* No code received. */
	NO_CODE_RECEIVED		= 0x00000000,
	TYPE_UNDEFINED			= 0xFFFFFFFF,

	/* Hardware reading request. */
	MCS_HardwareReq			= 0x0801f858,
	/* Command code to connect to the PCB. */
	MCS_ConnectTo			= 0xD9F2D9F7,

	/* Mother PCB commands (general command set). */
	MCS_GETSysInfo			= 0x6135E130,
	MCS_GETDeviceList		= 0x1019F501,
	MCS_SETEthAddr			= 0xC4159A09,
	MCS_SETAddr				= 0xAA582F78,
	MCS_ENBip				= 0x7A984D73,
	MCS_DISBip				= 0xFCA6393B,
	MCS_GetParams			= 0xfdcc0d08,
	MCS_SetParams			= 0xc2cd4d28,
	MCS_SaveParams			= 0x5a25bd71,
	MCS_RestoreParams		= 0x9f558a84,
	MCS_SyncDevices			= 0x8792dc7c,

	/* FPGA FIFO commands. */
	FPGA_FifoReset			= 0xd38c60eb,
	FPGA_GetSampleNumbers	= 0x46be02f9,
	FPGA_GetFifo0samples	= 0xf61986a2,
	FPGA_GetFifo1samples	= 0x83a7c015,
	FPGA_GetTSampleNumbers	= 0x8ab39fa5,
	FPGA_GetTSamples		= 0x56bc5e27,

	/* FPGA configuration commands. */
	FPGA_SoftwareStart		= 0xdf293ee9,
	FPGA_SyncRegisters		= 0x2dfe7610,
	FPGA_SetDefaulParams	= 0xfc0fedb3,
	FPGA_SetCtrlReg 		= 0x865b5ba0,
	FPGA_SetPeriodLimiter	= 0x7ae31452,
	FPGA_SetFStartTim		= 0x6a175534,
	FPGA_SetSStartTim		= 0xfe56a20a,
	FPGA_SetFStopTim		= 0xec709233,
	FPGA_SetSStopTim		= 0x0e55e89a,//
	FPGA_SetThrStartHigh	= 0xea2e5b92,
	FPGA_SetThrStartLow		= 0x213068ba,
	FPGA_SetThr0Stop		= 0x280ba17b,
	FPGA_SetThr1Stop		= 0x3fd89026,


	/* Global command set. */
	GCS_EnDisDevice			= 0x56b15d35,
	GCS_VIEWAddr			= 0xE77CE3C0,

	/* Broadcast command set. */
	BCS_EnDisDevice			= 0xc5e30e37,
	BCS_VIEWAddr			= 0x522652A6,


	/* Unicast command set. */
	UCS_GETInfo				= 0x8EE72275,
	UCS_EnDisDevice			= 0xf019b7af,
	UCS_SETAddr				= 0x827DBE96,
	UCS_LOCKAddr			= 0xB8B37E88,

	/* Command prefixes. */
	FPGA					= 0x49e4e69d,
	MCS						= 0x79823B50,
	GCS						= 0x1700711E,
	BCS						= 0x20415439,
	UCS						= 0xEF8523B8,
	AGU						= 0x08f37915,


} ALL_CMD_SETS_t;

/*
	Device subtype list.
*/
typedef enum {
	/* Generator subtypes. */
	AGU0,

	/* No device available. */
	NO_DEVICE_AVAILABLE = 0xFF


} DEV_SUBTYPE_t;

/*
	@brief HVU states.
*/
typedef enum {

	DEV_OFF,
	DEV_ON,
	STATE_UNDEFINED = 0xFF

} DEV_STATE_t;

/*
 * @brief General constants.
 */
typedef enum {

	/* MAX and MIN values for an external devices. */
	SGU_MIN_AMPL			= 0,
	SGU_MAX_AMPL			= 1024,

	SGU_MIN_FREQ			= 1000000,
	SGU_MAX_FREQ			= 250000000,

	SGU_MIN_PHASE			= 0,
	SGU_MAX_PHASE			= 360


} DEV_CONST_t;

/*
 * @brief Location of user data in the EEP memory.
 */
typedef enum {
	/* Address location. */
	DEV_ADDR_BYTE 		= 0,
	DEV_ADDR_PAGE 		= 0,

	/* Generator data. */
	GEN_DATA_BYTE 		= 0,
	GEN_DATA_PAGE 		= 1

} DEV_EXT_MEM_DATA_t;

/*
 * @brief Led channel definitions.
 */
typedef enum {

	CH1 = 1,
	CH2, CH3,
	CH4, CH5,
	CH6, CH_ALL

} LED_CH_t;

/*
 * @brief
 */
typedef struct {
	char state[5];
	char dev_id[11];
	char dev_subid[11];

} DEV_CharBuff_t;

/*
 * @brief State of LED channels on the front panel.
 */
typedef struct {

	uint8_t Channel;
	uint8_t State;

} LedChData_t;

/*
 * @brief Device list struct.
 */
typedef struct {
	/* Type of devices. */
	union {
		uint32_t DevType[MAX_UPCB_NUMBER];
		struct {
			uint32_t Device_1;
			uint32_t Device_2;
			uint32_t Device_3;
			uint32_t Device_4;
			uint32_t Device_5;
			uint32_t Device_6;
		};
	};
	/* Subtype of devices. */
	union {
		uint8_t DevSubType[MAX_UPCB_NUMBER];
		struct {
			uint8_t SubDevice_1;
			uint8_t SubDevice_2;
			uint8_t SubDevice_3;
			uint8_t SubDevice_4;
			uint8_t SubDevice_5;
			uint8_t SubDevice_6;
		};
	};

} DEV_List_t;

/*
 * @brief Generators' data.
 */
typedef struct {
	/* Generator's data. */
	float Phase[4];
	float Amplitude[4];
	float Frequency[4];

} DEV_GenData_t;

/*
 * @brief Define device electronic signature struct.
 */
typedef struct __attribute__((aligned(1), packed)) {

	union {
		__IO uint32_t Id_0_31;
		__IO uint32_t Id_XY_Coordinates;
	} XYCoordinates;

	union {
		__IO uint32_t Id_32_63;
		struct {
			__IO uint32_t WAF_NUM_0_7 	: 8;
			__IO uint32_t LOT_NUM_0_23 	: 24;
		};
	} WafLot;

	union {
		__IO uint32_t Id_64_95;
		__IO uint32_t Id_LotNum_24_55;
	} LotNum;

} STM32DevId_t;

/* Electronic device signature on the memory MAP. */
#define STM32_EL_DEV_SIG  ((STM32DevId_t *) UID_BASE)
/* Flash size. */
#define STM32_FLASH_SIZE ((uint16_t *) FLASHSIZE_BASE)

/* Public function prototypes. */
uint32_t* MCS_MakeCmdCodes(void);
DEV_List_t* DEV_GetType(void);
DEV_GenData_t* DEV_GetGenData(void);
uint8_t DEV_DecToStateByte(uint32_t DecValue);
void DEV_WriteDevType(uint8_t dev_channel, uint32_t dev_type, uint8_t dev_subtype);
void DEV_NameDecode(uint8_t State, uint32_t DevType, uint8_t DevSubtype, DEV_CharBuff_t* CharBuff);

void DEV_UpdateList();
void DEV_UpdateParams();
void DEV_UpdateState(uint32_t dev_type, uint32_t sprefix, uint8_t address);


void DEV_RWGenEepAll(uint8_t ReadWrite);

void DEV_GiveSoftSyncPulse(void);

void BTN_SWPress(void);
void LED_Ctrl(uint8_t channel, uint8_t state);

#endif /* COMMON_H_ */
