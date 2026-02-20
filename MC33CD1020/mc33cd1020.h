/*
 * @brief Common header for MC33CD1020.
 * Created 02.12.26 by asw3005. 
 *
 **/

#ifndef MC33CD1020_H_
#define MC33CD1020_H_

#include <stdint.h>
#ifndef NULL
#define NULL (void*)0
#endif /* NULL */


#include "stm32g431xx.h"

/* GPIO configuration. */
#define MC33CD1020_CS_Pin		GPIO_PIN_12
#define MC33CD1020_CS_GPIO_Port	GPIOB

#define MC33CD1020_READ_SEQ 	0x00
#define MC33CD1020_WRITE_SEQ 	0x01
#define MC33CD1020_DEFWRITE		0x01

/*
 * @brief 
 *
 **/

/*
 * @brief Register map.
 *
 **/
typedef enum {
	
	MC33CD1020_SPI_CHECK 				= 0x00,
	MC33CD1020_DEV_CFG 					= 0x02,
	MC33CD1020_TRISTATE_SP 				= 0x04,
	MC33CD1020_TRISTATE_SG 				= 0x06,
	MC33CD1020_WETTCURRENT_LVL_SP 		= 0x08,
	MC33CD1020_WETTCURRENT_LVL_SGR0 	= 0x0A,
	MC33CD1020_WETTCURRENT_LVL_SGR1		= 0x0C,
	MC33CD1020_WETTCURRENT_CONTEN_SP 	= 0x16,
	MC33CD1020_WETTCURRENT_CONTEN_SG 	= 0x18,
	MC33CD1020_INTEN_SP 				= 0x1A,
	MC33CD1020_INTEN_SG 				= 0x1C,
	MC33CD1020_LOWPWRMODE_CFG 			= 0x1E,
	MC33CD1020_WAKEUPEN_SP 				= 0x20,
	MC33CD1020_WAKEUPEN_SG 				= 0x22,
	MC33CD1020_LPMCMPONLY_SP 			= 0x24,
	MC33CD1020_LPMCMPONLY_SG 			= 0x26,
	MC33CD1020_LPMVOLTAGETHR_SP 		= 0x28,
	MC33CD1020_LPMVOLTAGETHR_SG 		= 0x2A,
	MC33CD1020_POLLCURRENTCFG_SP 		= 0x2C,
	MC33CD1020_POLLCURRENTCFG_SG 		= 0x2E,
	MC33CD1020_ENTERLPM 				= 0x39,
	MC33CD1020_AMUXCHSEL_SPI 			= 0x3A,
	MC33CD1020_READSWSTAT 				= 0x3E,
	MC33CD1020_FAULTSTAT 				= 0x42,
	MC33CD1020_INT_PULSE_REQ 			= 0x47,
	MC33CD1020_RST 						= 0x49
	
} MC33CD1020_REG_MAP_t;





/*
 * @brief Data exchange function typedefs.
 *
 **/
typedef void(*delay_fptr)(uint32_t delay);
typedef void(*spi_txrx_fptr)(uint8_t *pData, uint8_t size);

/*
 * @brief MISO return word.
 *
 **/
 typedef union {

	struct {
		uint32_t RetWord;
	};

	struct {
		uint32_t RESERVED21_0 		: 22;
		uint32_t INT_FLG 			: 1;
		uint32_t FAULT_STATUS		: 1;
		/* 7bit address + RW. */
		uint32_t REG_ADDR_RW0		: 8;
	};

	struct {
		uint32_t REG_DATA 			: 24;
		/* 7bit address + RW. */
		uint32_t REG_ADDR_RW1		: 8;
	};

 } MC33CD1020_RetWord_t;

/*
 * @brief Device configuration universal SP type register.
 *
 **/
 typedef union {

	struct {
		uint32_t UniSPReg;
	};

	struct {
		/* SP0 - SP7. */
		uint32_t SP0 				: 1;
		uint32_t SP1 				: 1;
		uint32_t SP2 				: 1;
		uint32_t SP3 				: 1;
		uint32_t SP4 				: 1;
		uint32_t SP5 				: 1;
		uint32_t SP6 				: 1;
		uint32_t SP7 				: 1;
		uint32_t RESERVED21_8 		: 14;
		uint32_t INT_FLG			: 1;
		uint32_t FAULT_STATUS		: 1;
		/* 7bit address + RW. */
		uint32_t REG_ADDR_RW		: 8;
	};

 } MC33CD1020_UniSP_t;

 /*
 * @brief Device configuration universal SG type register.
 *
 **/
 typedef union {

	struct {
		uint32_t UniSGReg;
	};

	struct {
		/* SG0 - SG13. */
		uint32_t SG0 				: 1;
		uint32_t SG1 				: 1;
		uint32_t SG2 				: 1;
		uint32_t SG3 				: 1;
		uint32_t SG4 				: 1;
		uint32_t SG5 				: 1;
		uint32_t SG6 				: 1;
		uint32_t SG7 				: 1;
		uint32_t SG8 				: 1;
		uint32_t SG9 				: 1;
		uint32_t SG10 				: 1;
		uint32_t SG11 				: 1;
		uint32_t SG12 				: 1;
		uint32_t SG13 				: 1;
		uint32_t RESERVED21_14 		: 8;
		uint32_t INT_FLG			: 1;
		uint32_t FAULT_STATUS		: 1;
		/* 7bit address + RW. */
		uint32_t REG_ADDR_RW		: 8;
	};

 } MC33CD1020_UniSG_t;

/*
 * @brief Device configuration register.
 *
 **/
 typedef union {

	struct {
		uint32_t DevCfgReg;
	};

	struct {
		/* SP0 - SP7 default 1. */
		uint32_t SP0 				: 1;
		uint32_t SP1 				: 1;
		uint32_t SP2 				: 1;
		uint32_t SP3 				: 1;
		uint32_t SP4 				: 1;
		uint32_t SP5 				: 1;
		uint32_t SP6 				: 1;
		uint32_t SP7 				: 1;
		uint32_t RESERVED9_8 		: 2;
		uint32_t INTB_OUT 			: 1;
		/* Default 1. */
		uint32_t WAKEB_VDDQCHECK 	: 1;
		uint32_t VBATP_OVDIS 		: 1;
		uint32_t SBPOLLTIME 		: 1;
		uint32_t RESERVED21_14 		: 8;
		uint32_t INT_FLG			: 1;
		uint32_t FAULT_STATUS		: 1;
		/* 7bit address + RW. */
		uint32_t REG_ADDR_RW		: 8;
	};

 } MC33CD1020_DevCfg_t;



/*
 * @brief Weting current level SP register.
 *
 **/
 typedef union {

	struct {
		uint32_t WettCurrentSPReg;
	};

	struct {
		/* SP0 - SP7 default 110. */
		uint32_t RESERVED0			: 1;
		uint32_t SP0 				: 2;
		uint32_t RESERVED3			: 1;
		uint32_t SP1 				: 2;
		uint32_t RESERVED6			: 1;
		uint32_t SP2 				: 2;
		uint32_t RESERVED9			: 1;
		uint32_t SP3 				: 2;
		uint32_t RESERVED12			: 1;
		uint32_t SP4 				: 2;
		uint32_t RESERVED15			: 1;
		uint32_t SP5 				: 2;
		uint32_t RESERVED18			: 1;
		uint32_t SP6 				: 2;
		uint32_t RESERVED21			: 1;
		uint32_t SP7 				: 2;
		/* 7bit address + RW. */
		uint32_t REG_ADDR_RW		: 8;
	};

 } MC33CD1020_WettCurrentSP_t;

/*
 * @brief Wetting current level SG register 0.
 *
 **/
 typedef union {

	struct {
		uint32_t WettCurrentSGReg0;
	};

	struct {
		/* SG0 - SG7 default 110. */
		uint32_t RESERVED0			: 1;
		uint32_t SG0 				: 2;
		uint32_t RESERVED3			: 1;
		uint32_t SG1 				: 2;
		uint32_t RESERVED6			: 1;
		uint32_t SG2 				: 2;
		uint32_t RESERVED9			: 1;
		uint32_t SG3 				: 2;
		uint32_t RESERVED12			: 1;
		uint32_t SG4 				: 2;
		uint32_t RESERVED15			: 1;
		uint32_t SG5 				: 2;
		uint32_t RESERVED18			: 1;
		uint32_t SG6 				: 2;
		uint32_t RESERVED21			: 1;
		uint32_t SG7 				: 2;
		/* 7bit address + RW. */
		uint8_t REG_ADDR_RW			: 8;
	};

 } MC33CD1020_WettCurrentSGReg0_t;

 /*
 * @brief Wetting current level SG register 1.
 *
 **/
 typedef union {

	struct {
		uint32_t WettCurrentSGReg1;
	};

	struct {
		/* SG8 - SG13 default 110. */
		uint32_t RESERVED0			: 1;
		uint32_t SG8 				: 2;
		uint32_t RESERVED3			: 1;
		uint32_t SG9 				: 2;
		uint32_t RESERVED6			: 1;
		uint32_t SG10 				: 2;
		uint32_t RESERVED9			: 1;
		uint32_t SG11 				: 2;
		uint32_t RESERVED12			: 1;
		uint32_t SG12 				: 2;
		uint32_t RESERVED15			: 1;
		uint32_t SG13 				: 2;
		uint32_t RESERVED23_18   	: 6;
		/* 7bit address + RW. */
		uint32_t REG_ADDR_RW		: 8;
	};

 } MC33CD1020_WettCurrentSGReg1_t;

 /*
 * @brief Low power mode register.
 *
 **/
 typedef union {

	struct {
		uint32_t LowPwrModeReg;
	};

	struct {
		/* POLL3_0 default 0x1111. */
		uint32_t POLL3_0 			: 4;
		uint32_t RESERVED23_8    	: 20;
		/* 7bit address + RW. */
		uint32_t REG_ADDR_RW		: 8;
	};

 } MC33CD1020_LowPwrMode_t;

 /*
 * @brief AMUX control register.
 *
 **/
 typedef union {

	struct {
		uint32_t AmuxCtrlReg;
	};

	struct {
		/* ASEL5_0 default 0. */
		uint32_t ASEL5_0 			: 6;
		uint32_t ASETT0				: 1;
		uint32_t RESERVED23_8    	: 17;
		/* 7bit address + RW. */
		uint32_t REG_ADDR_RW		: 8;
	};

 } MC33CD1020_AmuxCtrl_t;

 /*
 * @brief Read switch status register.
 *
 **/
 typedef union {

	struct {
		uint32_t SwStatusReg;
	};

	struct {
		/* SG0 - SG13. */
		uint32_t SG0 				: 1;
		uint32_t SG1 				: 1;
		uint32_t SG2 				: 1;
		uint32_t SG3 				: 1;
		uint32_t SG4 				: 1;
		uint32_t SG5 				: 1;
		uint32_t SG6 				: 1;
		uint32_t SG7 				: 1;
		uint32_t SG8 				: 1;
		uint32_t SG9 				: 1;
		uint32_t SG10 				: 1;
		uint32_t SG11 				: 1;
		uint32_t SG12 				: 1;
		uint32_t SG13 				: 1;
		/* SP0 - SP7. */
		uint32_t SP0 				: 1;
		uint32_t SP1 				: 1;
		uint32_t SP2 				: 1;
		uint32_t SP3 				: 1;
		uint32_t SP4 				: 1;
		uint32_t SP5 				: 1;
		uint32_t SP6 				: 1;
		uint32_t SP7 				: 1;
		
		uint32_t INT_FLG 			: 1;
		uint32_t FAULT_STATUS		: 1;
		/* 7bit address + R. */
		uint32_t REG_ADDR_RW		: 8;
	};

 } MC33CD1020_SwStatusRead_t;

  /*
 * @brief Fault status register.
 *
 **/
 typedef union {

	struct {
		uint32_t FaultStatusReg;
	};

	struct {
		/*  */
		uint32_t POR 				: 1;
		uint32_t SPI_WAKE			: 1;
		uint32_t WAKEB_WAKE			: 1;
		uint32_t INTB_WAKE			: 1;
		uint32_t OT  				: 1;
		uint32_t TEMP_FLG			: 1;
		uint32_t OV  				: 1;
		uint32_t UV  				: 1;
		uint32_t RESERVED16			: 1;
		uint32_t HASH_FAULT			: 1;
		uint32_t SPI_ERR			: 1;
		uint32_t RESERVED21_11		: 1;
		/* Default 1. */
		uint32_t INT_FLG 			: 1;
		uint32_t RESERVED23  		: 1;
		/* 7bit address + R. */
		uint32_t REG_ADDR_RW		: 8;
	};

 } MC33CD1020_FaultStatus_t;

/*
 * @brief SPI tx/rx data fields.
 *
 **/
 typedef union {

	struct {

		uint32_t RegData;
	};

	struct {

		uint8_t CmdWord;
		uint8_t RegDataH_LSB;
		uint8_t RegDataL_MSB;
		uint8_t RegDataL_LSB;
	};

 } MC33CD1020_TxRxData_t;


/*
 * @brief General data instance struct.
 *
 **/
typedef struct {

	delay_fptr delay;
	spi_txrx_fptr spi_tx;
	spi_txrx_fptr spi_rx;

} MC33CD1020_GInst_t;


/* Public function prototypes. */
uint32_t MC33CD1020_SPICheck(void);
MC33CD1020_DevCfg_t MC33CD1020_DevCfg(uint8_t rw_bit, 
											uint8_t sp0, uint8_t sp1, uint8_t sp2, uint8_t sp3, 
											uint8_t sp4, uint8_t sp5, uint8_t sp6, uint8_t sp7,
											uint8_t intb_out, uint8_t wakeb_vddqcheck,
											uint8_t vbat_ovdis, uint8_t sbpoll_time
											);
 MC33CD1020_UniSP_t MC33CD1020_TriStateSP(uint8_t rw_bit, 
											uint8_t sp0, uint8_t sp1, uint8_t sp2, uint8_t sp3, 
											uint8_t sp4, uint8_t sp5, uint8_t sp6, uint8_t sp7
											);
 MC33CD1020_UniSG_t MC33CD1020_TriStateSG(uint8_t rw_bit, 
											uint8_t sg0, uint8_t sg1, uint8_t sg2, uint8_t sg3, 
											uint8_t sg4, uint8_t sg5, uint8_t sg6, uint8_t sg7,
											uint8_t sg8, uint8_t sg9, uint8_t sg10, uint8_t sg11,
											uint8_t sg12, uint8_t sg13
											);
 MC33CD1020_WettCurrentSP_t MC33CD1020_WettCurrentSP(uint8_t rw_bit, 
											uint8_t wettcurrent_sp0, uint8_t wettcurrent_sp1, uint8_t wettcurrent_sp2, uint8_t wettcurrent_sp3,
											uint8_t wettcurrent_sp4, uint8_t wettcurrent_sp5, uint8_t wettcurrent_sp6, uint8_t wettcurrent_sp7
											);
 MC33CD1020_WettCurrentSGReg0_t MC33CD1020_WettCurrentSGReg0(uint8_t rw_bit, 
											uint8_t wettcurrent_sg0, uint8_t wettcurrent_sg1, uint8_t wettcurrent_sg2, uint8_t wettcurrent_sg3,
											uint8_t wettcurrent_sg4, uint8_t wettcurrent_sg5, uint8_t wettcurrent_sg6, uint8_t wettcurrent_sg7
											);
 MC33CD1020_WettCurrentSGReg1_t MC33CD1020_WettCurrentSGReg1(uint8_t rw_bit, 
											uint8_t wettcurrent_sg8, uint8_t wettcurrent_sg9, uint8_t wettcurrent_sg10, uint8_t wettcurrent_sg11,
											uint8_t wettcurrent_sg12, uint8_t wettcurrent_sg13
											);																					 
 MC33CD1020_UniSP_t MC33CD1020_ContWettCurrentSP(uint8_t rw_bit, 
											uint8_t contwett_current_sp0, uint8_t contwett_current_sp1, uint8_t contwett_current_sp2, uint8_t contwett_current_sp3, 
											uint8_t contwett_current_sp4, uint8_t contwett_current_sp5, uint8_t contwett_current_sp6, uint8_t contwett_current_sp7
											);
MC33CD1020_UniSG_t MC33CD1020_ContWettCurrentSG(uint8_t rw_bit, 
											uint8_t contwett_current_sg0, uint8_t contwett_current_sg1,	uint8_t contwett_current_sg2, uint8_t contwett_current_sg3,
											uint8_t contwett_current_sg4, uint8_t contwett_current_sg5, uint8_t contwett_current_sg6, uint8_t contwett_current_sg7,
											uint8_t contwett_current_sg8, uint8_t contwett_current_sg9, uint8_t contwett_current_sg10, uint8_t contwett_current_sg11, 
											uint8_t contwett_current_sg12, uint8_t contwett_current_sg13
											);


#endif /* MC33CD1020_H_ */










