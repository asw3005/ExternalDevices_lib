/*
	@brief Header of nRF24l01 module.
	Created 08.31.22 by asw3005.
*/

#ifndef NRF24L01_H_
#define NRF24L01_H_

#include "stm32f429xx.h"

#ifndef NULL
#define NULL (void *)0	
#endif //NULL

#define USE_INTERRUPT_PIN

/* Port's definations. */
#define INTERRUPT_INPUT_PORT	GPIOB
#define INTERRUPT_INPUT_PIN		GPIO_PIN_9

#define CE_GPIO_PORT			GPIOB
#define CE_GPIO_PIN				GPIO_PIN_8

#define CS_GPIO_PORT			GPIOB
#define CS_GPIO_PIN				GPIO_PIN_6

/*
	@brief Memory map.
*/
typedef enum {
	/* Configuration Register. */
	NRF24L01_CONFIG,
	/* Enable ‘Auto Acknowledgment’ function. Disable this functionality to be compatible with nRF2401. */
	NRF24L01_EN_AA,
	/* Enabled RX Addresses. */
	NRF24L01_EN_RXADDR,
	/* Setup of Address Widths (common for all data pipes). */
	NRF24L01_SETUP_AW,
	/* Setup of Automatic Retransmission. */
	NRF24L01_SETUP_RETR,
	/* RF Channel. */
	NRF24L01_RF_CH,
	/* RF Setup Register. */
	NRF24L01_RF_SETUP,
	/* Status Register (In parallel to the SPI instruction word applied on the MOSI pin, the STATUS register is shifted serially out on the 
	MISO pin). */
	NRF24L01_STATUS,
	/* Transmit observe register. */
	NRF24L01_OBSERVE_TX,
	/* Carrier Detect. */
	NRF24L01_CD,
	/* Receive address data pipe 0. 5 Bytes maximum length (LSByte is written first. Write the number of bytes defined by SETUP_AW). */
	NRF24L01_RX_ADDR_P0,
	/* Transmit address. Used for a PTX device only (LSByte is written first). Set RX_ADDR_P0 equal to this address to handle automatic 
	acknowledge if this is a PTX device with Enhanced ShockBurst™ enabled. */
	NRF24L01_TX_ADDR		= 0x10,
	/* Number of bytes in RX payload in data pipe 0. */
	NRF24L01_RX_PW_P0,
	/* FIFO Status Register. */
	NRF24L01_FIFO_STATUS	= 0x17

} NRF24L01_MEMORY_MAP_t;

/*
	@brief Device's command set.
*/
typedef enum {
	/* 000A AAAA. Read registers. AAAAA = 5 bit Memory Map Address. */
	NRF24L01_R_REGISTER		= 0x00,
	/* 001A AAAA. Write registers. AAAAA = 5 bit Memory Map Address. Executable in power down or standby modes only. */
	NRF24L01_W_REGISTER		= 0x01,
	/* Read RX-payload: 1 – 32 bytes. A read operation will always start at byte 0. Payload will be deleted from FIFO after it is read. 
	Used in RX mode. */
	NRF24L01_R_RX_PAYLOAD	= 0x61,
	/* Write TX-payload: 1 – 32 bytes. A write operation will always start at byte 0. Used in TX mode. */
	NRF24L01_W_TX_PAYLOAD	= 0xA0,
	/* Flush TX FIFO, used in TX mode. */
	NRF24L01_FLUSH_TX		= 0xE1,
	/* Flush RX FIFO, used in RX mode. Should not be executed during transmission of acknowledge, i.e. acknowledge package will not be 
	completed. */
	NRF24L01_FLUSH_RX		= 0xE2,
	/* Reuse last sent payload. Packets will be repeatedly resent as long as CE is high. TX payload reuse is active until W_TX_PAYLOAD or
	FLUSH TX is executed. TX payload reuse must not be activated or deactivated during package transmission. Used for a PTX device. */
	NRF24L01_REUSE_TX_PL	= 0xE3,
	/* No Operation. Might be used to read the STATUS register. */
	NRF24L01_NOP			= 0xFF

} NRF24L01_CMD_SET_t;

/*
	@brief Constants.
*/
typedef enum {

	NRF24L01_SEND_OK,
	NRF24L01_SEND_ERR,

	NRF24L01_CMD_BYTE_AMOUNT		= 1,
	NRF24L01_FULL_PAYLOAD_AMOUNT	= 32,
	NRF24L01_LOW_ADDR_AMOUNT		= 1,
	NRF24L01_FULL_ADDR_AMOUNT		= 5,
	NRF24L01_FULL_RXTX_ADDR_AMOUNT	= 19,

	NRF24L01_DELAY_BEFORE_CSHIGH	= 1

} NRF24L01_CONST_t;

/*
	@brief Pipes' numbers.
*/
typedef enum {

	NRF24L01_PIPE0,
	NRF24L01_PIPE1,
	NRF24L01_PIPE2,
	NRF24L01_PIPE3,
	NRF24L01_PIPE4,
	NRF24L01_PIPE5

} NRF24L01_PIPE_NUMBER_t;

/*
	@brief Chip select pin states.
*/
typedef enum {

	NRF24L01_CS_ACTIVE		= 1,
	NRF24L01_CS_NOTACTIVE	= 0,

	NRF24L01_CE_ACTIVE		= 1,
	NRF24L01_CE_NOTACTIVE	= 0

} NRF24L01_CSPIN_STATE_t;

/*
	@brief Delay function type.
*/
typedef void (*delay_fptr)(uint32_t delay);

/*
	@brief Chip select control function pointer type.
*/
typedef void (*pin_ctrl_fptr)(uint8_t state);

/*
	@brief Data Rx/Tx function pointer type.
*/
typedef void (*rxtx_data_fptr)(uint8_t* pdata, uint16_t size);

/*
	@brief Read or write command format.
*/
typedef union {

	uint8_t RWCmd;
	struct {
		/* 5 bit memory map address. */
		uint8_t MEM_MAP_ADDR	: 5;
		/* NRF24L01_R_REGISTER or NRF24L01_W_REGISTER command. */
		uint8_t RW_CMD			: 3;
	};
} NRF24L01_RWCmd_t;

/*
	@brief Configuration register.
*/
typedef union {

	uint8_t CfgReg;
	struct {
		/* 1: PRX, 0: PTX. */
		uint8_t PRIM_RX		: 1;
		/* 1: POWER UP, 0:POWER DOWN. */
		uint8_t PWR_UP		: 1;
		/* CRC encoding scheme '0' - 1 byte, '1' – 2 bytes. */
		uint8_t CRCO		: 1;
		/* Enable CRC. Forced high if one of the bits in the EN_AA is high. Default value is 1. */
		uint8_t EN_CRC		: 1;
		/* Mask interrupt caused by MAX_RT 1: Interrupt not reflected on the IRQ pin, 0: Reflect MAX_RT as active low interrupt on the IRQ 
		pin. */
		uint8_t MASK_MAX_RT : 1;
		/* Mask interrupt caused by TX_DS 1: Interrupt not reflected on the IRQ pin, 0: Reflect TX_DS as active low interrupt on the IRQ pin. */
		uint8_t MASK_TX_DS	: 1;
		/* Mask interrupt caused by RX_DR 1: Interrupt not reflected on the IRQ pin, 0: Reflect RX_DR as active low interrupt on the IRQ pin. */
		uint8_t MASK_RX_DR	: 1;
		/* Only '0' allowed. */
		uint8_t RESERVED	: 1;
	};
} NRF24L01_CfgReg_t;

/*
	@brief Enable auto ack for data pipes.
*/
typedef union {

	uint8_t AutoAckReg;
	struct {
		/* Enable auto ackfor the pipe X, where X is 0 to 5. Default value is 1 for all. */
		uint8_t ENAA_P0		: 1;
		uint8_t ENAA_P1		: 1;
		uint8_t ENAA_P2		: 1;
		uint8_t ENAA_P3		: 1;
		uint8_t ENAA_P4		: 1;
		uint8_t ENAA_P5		: 1;
		/* Only '00' allowed. */
		uint8_t RESERVED	: 2;
	};
} NRF24L01_AutoAck_t;

/*
	@brief Enable RX addresses.
*/
typedef union {

	uint8_t EnRxAddrReg;
	struct {
		/* Enable data pipe X, where X is 0 to 5.  Default value is 1 for P0 and P1. */
		uint8_t ERX_P0		: 1;
		uint8_t ERX_P1		: 1;
		uint8_t ERX_P2		: 1;
		uint8_t ERX_P3		: 1;
		uint8_t ERX_P4		: 1;
		uint8_t ERX_P5		: 1;
		/* Only '00' allowed. */
		uint8_t RESERVED	: 2;
	};
} NRF24L01_EnRxAddr_t;

/*
	@brief Setup addresses' width. 
*/
typedef union {

	uint8_t AddrWidthReg;
	struct {
		/* RX/TX Address field width '00' - Illegal,
									 '01' - 3 bytes,
									 '10' - 4 bytes,
									 '11' – 5 bytes.
		LSByte will be used if address width below 5 bytes. Default value is 11. */
		uint8_t AW			: 2;
		/* Only '00' allowed. */
		uint8_t RESERVED	: 6;
	};
} NRF24L01_AddrWidth_t;

/*
	@brief Automatic retransmission setups.
*/
typedef union {

	uint8_t AutoRetransReg;
	struct {
		/* Auto Retransmit Count ‘0000’ –Re-Transmit disabled,
								 ‘0001’ – Up to 1 Re-Transmit on fail of AA,
								  ……,
								 ‘1111’ – Up to 15 Re-Transmit on fail of AA.
		Default value is 0011. */
		uint8_t ARC : 4;
		/* Auto Re-transmit Delay ‘0000’ – Wait 250+86uS, 
								  ‘0001’ – Wait 500+86uS,
								  ‘0010’ – Wait 750+86uS,
								   ……..,
								  ‘1111’ – Wait 4000+86uS.
		Delay defined from end of transmission to start of next transmission. */
		uint8_t ARD : 4;
	};
} NRF24L01_AutoRetans_t;

/*
	@brief Frequency channel setup.
*/
typedef union {

	uint8_t RFChannelReg;
	struct {
		/* Sets the frequency channel nRF24L01 operates on. Default value is 10. */
		uint8_t RF_CH		: 7;
		/* Only '00' allowed. */
		uint8_t RESERVED	: 1;
	};
} NRF24L01_RFChannel_t;

/*
	@brief RF setup register.
*/
typedef union {

	uint8_t RFSetupReg;
	struct {
		/* Setup LNA gain. Default value is 1. */
		uint8_t LNA_HCURR	: 1;
		/* Set RF output power in TX mode '00' – -18 dBm,
										  '01' – -12 dBm,
										  '10' – -6 dBm,
										  '11' – 0 dBm. 
		Default value is 11. */
		uint8_t RF_PWR		: 2;
		/* Data Rate ‘0’ – 1 Mbps,
					 ‘1’ – 2 Mbps. 
		Default value is 1. */
		uint8_t RF_DR		: 1;
		/* Force PLL lock signal. Only used in test. Should be set to 0. */
		uint8_t PLL_LOCK	: 1;
		/* Only '00' allowed. */
		uint8_t RESERVED	: 3;
	};
} NRF24L01_RFSetup_t;

/*
	@brief Status register.
*/
typedef union {

	uint8_t StatusReg;
	struct {
		/* TX FIFO full flag. 1: TX FIFO full. 0: Available locations in TX FIFO. */
		uint8_t TX_FULL		: 1;
		/* Data pipe number for the payload available for reading from RX_FIFO 000-101: Data Pipe Number,
																			   110: Not Used,
																			   111: RX FIFO Empty.
		Default value is 111. */
		uint8_t RX_P_NO		: 3;
		/* Maximum number of TX retries interrupt. Write 1 to clear bit. If MAX_RT is set it must be cleared to enable further communication. */
		uint8_t MAX_RT		: 1;
		/* Data Sent TX FIFO interrupt. Set high when packet sent on TX. If AUTO_ACK is activated, this bit will be set high only when ACK is received.
		Write 1 to clear bit. */
		uint8_t TX_DS		: 1;
		/* Data Ready RX FIFO interrupt. Set high when new data arrives RX FIFO. Write 1 to clear bit. 
		The Data Ready interrupt is set by a new packet arrival event. The procedure for handling this interrupt should be: 
		1) read payload via SPI, 
		2) clear RX_DR interrupt, 
		3) read FIFO_STATUS to check if there are more payloads available in RX FIFO, 
		4) if there are more data in RX FIFO, repeat from 1). */
		uint8_t RX_DR		: 1;
		/* Only '00' allowed. */
		uint8_t RESERVED	: 1;
	};
} NRF24L01_Status_t;

/*
	@brief Transmit observing.
*/
typedef union {

	uint8_t ObserveTxReg;
	struct {
		/* Count resent packets. The counter is reset when transmission of a new packet starts. */
		uint8_t ARC_CNT		: 4;
		/* Count lost packets. The counter is overflow protected to 15, and discontinue at max until reset. The counter is reset by writing to RF_CH. */
		uint8_t PLOS_CNT	: 4;
	};
} NRF24L01_Observe_t;

/*
	@brief Carrier detect.
*/
typedef union {

	uint8_t CarrierDetReg;
	struct {
		/* Carrier detect bit. */
		uint8_t CD			: 1;
		/*  */
		uint8_t RESERVED	: 7;
	};
} NRF24L01_CarrierDet_t;

/*
	@brief Number of bytes. 
*/
typedef union {

	uint8_t NumberOfBytesReg;
	struct {
		/* Number of bytes in RX payload in data pipe 0 (1 to 32 bytes).
		0 Pipe not used,
		1 = 1 byte,
		…,
		32 = 32 bytes. */
		uint8_t RX_PW_Px : 6;
		/* Only '00' allowed. */
		uint8_t RESERVED : 2;
	};
} NRF24L01_NumberBytes_t;

/*
	@brief FIFO status.
*/
typedef union {

	uint8_t FIFOStatusReg;
	struct {
		/* RX FIFO empty flag. 1: RX FIFO empty.
							   0: Data in RX FIFO. 
   	    Default value is 1. */
		uint8_t RX_EMPTY	: 1;
		/* RX FIFO full flag. 1: RX FIFO full. 
							  0: Available locations in RX FIFO. */
		uint8_t RX_FULL		: 1;
		/* Only '00' allowed. */
		uint8_t RESERVED0	: 2;
		/* TX FIFO empty flag. 1: TX FIFO empty.
							   0: Data in TX FIFO. */
		uint8_t TX_EMPTY	: 1;
		/* TX FIFO full flag. 1: TX FIFO full. 
							  0: Available locations in TX FIFO. */
		uint8_t TX_FULL		: 1;
		/* Reuse last sent data packet if set high. The packet will be repeatedly resent as long as CE is high. TX_REUSE is set by the SPI instruction 
		REUSE_TX_PL, and is reset by the SPI instructions W_TX_PAYLOAD or FLUSH TX. */
		uint8_t TX_REUSE	: 1;
		/* Only '00' allowed. */
		uint8_t RESERVED1	: 1;
	};
} NRF24L01_FIFOStatus_t;

/*
	@brief Address bit field.
*/
typedef struct __attribute__((aligned(1), packed)) {

	union {
		uint8_t AddrByte[5];
		struct {
			uint8_t ADDR_LOW_BITS	: 8;
			uint32_t ADDR_HIGH_BITS : 32;
		};
		struct {
			uint64_t ADDR_ALL_BITS	: 40;
		};
	};
} NRF24L01_Addr_t;

/*
	@brief Set of addresses.
*/
typedef struct __attribute__((aligned(1), packed)) {
	/* Data pipe 0 has a unique 40 bit configurable address. */
	/* In the PTX device data pipe 0 is used to received the acknowledgement, and therefore the receive address for data pipe 0 has to be equal
	to the transmit address to be able to receive the acknowledgement. Default value is 0xE7E7E7E7E7. */
	NRF24L01_Addr_t RxAddrPipe0;
	/* Each of data pipe 1-5 has an 8 bit unique address and shares the 32 most significant address bits. Default value is 0xC2C2C2C2C2. */
	NRF24L01_Addr_t RxAddrPipe1;
	/* Default value is C3. */
	uint8_t RxAddrPipe2;
	/* Default value is C4. */
	uint8_t RxAddrPipe3;
	/* Default value is C5. */
	uint8_t RxAddrPipe4;
	/* Default value is C6. */
	uint8_t RxAddrPipe5;
	/* Default value is 0xE7E7E7E7E7. */
	NRF24L01_Addr_t TxAddr;

} NRF24L01_SetAddr_t;

/*
	@brief Data payload.
*/
typedef struct {

	uint8_t Size;
	uint8_t Data[NRF24L01_CMD_BYTE_AMOUNT + NRF24L01_FULL_PAYLOAD_AMOUNT];

} NRF24L01_TxRxData_t;

/*
	@brief General data struct
*/
typedef struct {

	delay_fptr delay_f;
	pin_ctrl_fptr cepin_ctrl_f;
	pin_ctrl_fptr cspin_ctrl_f;
	rxtx_data_fptr spi_rx_data_f;
	rxtx_data_fptr spi_tx_data_f;

} NRF24L01_GInst_t;

/* Public function prototypes. */
void NRF24L01_Init(NRF24L01_GInst_t* device);

void NRF24L01_SetPipeTxAddr(NRF24L01_GInst_t* device, uint64_t addr);
uint8_t NRF24L01_SetPipeRxAddr(NRF24L01_GInst_t* device, NRF24L01_PIPE_NUMBER_t pipe, uint64_t addr);
void NRF24L01_SetPipeAddrBurst(NRF24L01_GInst_t* device, uint64_t pipe0_addr, uint64_t pipe1_addr, uint8_t pipe2_addr, uint8_t pipe3_addr,
	uint8_t pipe4_addr, uint8_t pipe5_addr, uint64_t tx_addr);

void NRF24L01_WritePayload(NRF24L01_GInst_t* device, uint8_t* pdata, uint8_t size);
NRF24L01_TxRxData_t* NRF24L01_ReadPayload(NRF24L01_GInst_t* device, uint8_t size);

NRF24L01_Observe_t NRF24L01_ReadObserveTx(NRF24L01_GInst_t* device);
uint8_t NRF24L01_ReadCarrierDet(NRF24L01_GInst_t* device);
NRF24L01_FIFOStatus_t NRF24L01_ReadFIFOStatus(NRF24L01_GInst_t* device);

NRF24L01_Status_t NRF24L01_ReadStatusV1(NRF24L01_GInst_t* device);
NRF24L01_Status_t NRF24L01_ReadStatusV2(NRF24L01_GInst_t* device);

void NRF24L01_FlushTx(NRF24L01_GInst_t* device);
void NRF24L01_FlushRx(NRF24L01_GInst_t* device);

void NRF24L01_ReuseTxPayload(NRF24L01_GInst_t* device);
void NRF24L01_WriteReg(NRF24L01_GInst_t* device, uint8_t reg_addr, uint8_t* reg_value, uint8_t size);
uint8_t NRF24L01_ReadReg(NRF24L01_GInst_t* device, uint8_t reg_addr);

void NRF24L01_GoStandby1(NRF24L01_GInst_t* device);
void NRF24L01_GoStandby2(NRF24L01_GInst_t* device);

void NRF24L01_TxStart(NRF24L01_GInst_t* device);
void NRF24L01_RxStart(NRF24L01_GInst_t* device);

/* Hardware dependent functions. */
void NRF24L01_Delay(uint32_t delay);
void SPI_CECtrl(NRF24L01_CSPIN_STATE_t state);
void SPI_CSCtrl(NRF24L01_CSPIN_STATE_t state);
void SPI_RxData(uint8_t* pdata, uint16_t size);
void SPI_TxData(uint8_t* pdata, uint16_t size);

#endif /* NRF24L01_H_ */
