/*
 * @brief Common header for TMC262C.
 * Created 02.12.26 by asw3005. 
 *
 **/

#ifndef TMC262C_H_
#define TMC262C_H_

#include <stdint.h>
#ifndef NULL
#define NULL (void*)0
#endif /* NULL */


#include "stm32g431xx.h"

/* GPIO configuration. */
#define TMC262C_EN_Pin 			GPIO_PIN_0
#define TMC262C_EN_GPIO_Port	GPIOC
#define TMC262C_DIR_Pin 		GPIO_PIN_1
#define TMC262C_DIR_GPIO_Port	GPIOC
#define TMC262C_STEP_Pin 		GPIO_PIN_2
#define TMC262C_STEP_GPIO_Port	GPIOC

#define TMC262C_CS_Pin 			GPIO_PIN_15
#define TMC262C_CS_GPIO_Port	GPIOA
#define TMC262C_SCK_Pin 		GPIO_PIN_10
#define TMC262C_SCK_GPIO_Port	GPIOC
#define TMC262C_MISO_Pin 		GPIO_PIN_11
#define TMC262C_MISO_GPIO_Port	GPIOC
#define TMC262C_MOSI_Pin 		GPIO_PIN_12
#define TMC262C_MOSI_GPIO_Port	GPIOC

/*
 * @brief 
 *
 **/

/*
 * @brief Register map.
 *
 **/
typedef enum {
	
	TMC262C_DRVCTRL,
	TMC262C_CHOPCONF = 0x04,
	TMC262C_SMARTEN,
	TMC262C_SGCSCONF,
	TMC262C_DRVCONF
	
} TMC262C_REG_MAP_t;

/*
 * @brief Register map.
 *
 **/
typedef enum {

	TMC262C_SMALL_TEST_R75,
	TMC262C_NEMA17_R75,
	TMC262C_NEMA17_R50

} TMC262C_MOTOR_PROFILE_t;

 /*
 * @brief Device configuration DRVCTRL type register (SDOFF = 0).
 *
 **/
 typedef union __attribute__((aligned(1), packed)) {

	struct {
		uint8_t DrvCtrl_LSB_H;
		uint8_t DrvCtrl_MSB_L;
		uint8_t DrvCtrl_LSB_L;
	};

	struct {
		/* MUST BE ZERO. */
		uint8_t RESERVED17_16	: 2;
		/* Reg address bits. */
		uint8_t REG_ADDR 		: 2;
		uint8_t RESERVED23_20 	: 4;

		uint8_t DEDGE 			: 1;
		uint8_t INTPOL 			: 1;
		/* MUST BE ZERO. */
		uint8_t RESERVED15_10	: 6;

		uint8_t MRES3_0 		: 4;
		/* MUST BE ZERO. */
		uint8_t RESERVED7_4		: 4;
	};

 } TMC262C_DrvCtrl_SDOFF0_t;

/*
 * @brief Device configuration DRVCTRL type register (SDOFF = 1).
 *
 **/
 typedef union __attribute__((aligned(1), packed)) {

	struct {
		uint8_t DrvCtrl_LSB_H;
		uint8_t DrvCtrl_MSB_L;
		uint8_t DrvCtrl_LSB_L;
	};

	struct {
		uint8_t CA7 			: 1;
		uint8_t PHA 			: 1;
		/* Reg address bits. */
		uint8_t REG_ADDR		: 2;
		uint8_t RESERVED24_20 	: 4;

		uint8_t PHB 			: 1;
		uint8_t CA6_0 			: 7;

		uint8_t CB7_0 			: 8;
	};

 } TMC262C_DrvCtrl_SDOFF1_t;

/*
 * @brief Device configuration CHOPCONF type register.
 *
 **/
 typedef union __attribute__((aligned(1), packed)) {

	struct {
		uint8_t ChopConf_LSB_H;
		uint8_t ChopConf_MSB_L;
		uint8_t ChopConf_LSB_L;
	};

	struct {
		uint8_t TBL1			: 1;
		/* Reg address bits. */
		uint8_t REG_ADDR		: 3;
		uint8_t RESERVED24_20 	: 4;

		uint8_t HEND3_1			: 3;
		uint8_t HDEC1_0			: 2;
		uint8_t RNDTF			: 1;
		uint8_t CHM 			: 1;
		uint8_t TBL0			: 1;

		uint8_t TOFF3_0			: 4;
		uint8_t HSTRT2_0		: 3;
		uint8_t HEND0			: 1;
	};

 } TMC262C_ChopConf_t;

 /*
 * @brief Device configuration SMARTEN type register.
 *
 **/
 typedef union __attribute__((aligned(1), packed)) {

	struct {
		uint8_t SmartEn_LSB_H;
		uint8_t SmartEn_MSB_L;
		uint8_t SmartEn_LSB_L;
	};

	struct {
		/* MUST BE ZERO. */
		uint8_t RESERVED16		: 1;
		/* Reg address bits. */
		uint8_t REG_ADDR		: 3;
		uint8_t RESERVED24_20 	: 4;

		uint8_t SEMAX3_0		: 4;
		/* MUST BE ZERO. */
		uint8_t RESERVED12 		: 1;
		uint8_t SEDN1_0			: 2;
		uint8_t SEIMIN			: 1;

		uint8_t SEMIN3_0		: 4;
		/* MUST BE ZERO. */
		uint8_t RESERVED4		: 1;
		uint8_t SEUP1_0			: 2;
		/* MUST BE ZERO. */
		uint8_t RESERVED7		: 1;
	};

 } TMC262C_SmartEn_t;

/*
 * @brief Device configuration SGCSCONF type register.
 *
 **/
 typedef union __attribute__((aligned(1), packed)) {

	struct {
		uint8_t SgcsConf_LSB_H;
		uint8_t SgcsConf_MSB_L;
		uint8_t SgcsConf_LSB_L;
	};

	struct {
		uint8_t SFILT			: 1;
		/* Reg address bits. */
		uint8_t REG_ADDR		: 3;
		uint8_t RESERVED24_20 	: 4;

		uint8_t SGT6_0			: 7;
		/* MUST BE ZERO. */
		uint8_t RESERVED15		: 1;

		uint8_t CS4_0			: 5;
		/* MUST BE ZERO. */
		uint8_t RESERVED7_5		: 3;
	};

 } TMC262C_SgcsConf_t;

/*
 * @brief Device configuration DRVCONF type register.
 *
 **/
 typedef union __attribute__((aligned(1), packed)) {

	struct {
		uint8_t DrvConf_LSB_H;
		uint8_t DrvConf_MSB_L;
		uint8_t DrvConf_LSB_L;
	};

	struct {
		uint8_t TST 			: 1;
		/* Reg address bits. */
		uint8_t REG_ADDR		: 3;
		uint8_t RESERVED24_20 	: 4;

		uint8_t TS2G1_0			: 2;
		uint8_t DISS2G			: 1;
		uint8_t SLP2			: 1;
		uint8_t SLPL1_0			: 2;
		uint8_t SLPH1_0 		: 2;

		uint8_t EN_S2VS			: 1;
		uint8_t EN_PFD			: 1;
		uint8_t SHRTSENS		: 1;
		uint8_t OTSENS			: 1;
		uint8_t RDSEL1_0		: 2;
		uint8_t VSENSE			: 1;
		uint8_t SDOFF			: 1;
	};

 } TMC262C_DrvConf_t;

 /*
 * @brief Device configuration read type register RDSEL00.
 *
 **/
 typedef union __attribute__((aligned(1), packed)) {

	struct {
		uint8_t ReadBack_MSB_H;
		uint8_t ReadBack_MSB_L;
		uint8_t ReadBack_LSB_L;
	};

	struct {
		/* RDSEL = 00. */
		uint8_t RDSEL00_MSTEP9_2	: 8;

		uint8_t RDSEL00_SHORTB_S2GB	: 1;
		uint8_t RDSEL00_OLA			: 1;
		uint8_t RDSEL00_OLB			: 1;
		uint8_t RDSEL00_STST		: 1;
		/* Reads as 00. */
		uint8_t RDSEL00_RESERVED9_8	: 2;
		uint8_t RDSEL00_MSTEP1_0	: 2;

		uint8_t RDSEL00_RESERVED3_0	: 4;
		uint8_t RDSEL00_SG			: 1;
		uint8_t RDSEL00_OT			: 1;
		uint8_t RDSEL00_OTPW		: 1;
		uint8_t RDSEL00_SHORTA_S2GA	: 1;
	};


 } TMC262C_ReadBackRDSEL00_t;	

/*
 * @brief Device configuration read type register RDSEL01.
 *
 **/
 typedef union __attribute__((aligned(1), packed)) {

	struct {
		uint8_t ReadBack_MSB_H;
		uint8_t ReadBack_MSB_L;
		uint8_t ReadBack_LSB_L;
	};

	struct {
		/* RDSEL = 01. */
		uint8_t RDSEL01_SG9_2		: 8;

		uint8_t RDSEL01_SHORTB_S2GB	: 1;
		uint8_t RDSEL01_OLA			: 1;
		uint8_t RDSEL01_OLB			: 1;
		uint8_t RDSEL01_STST		: 1;
		/* Reads as 00. */
		uint8_t RDSEL01_RESERVED9_8	: 2;
		uint8_t RDSEL01_SG1_0		: 2;

		uint8_t RDSEL01_RESERVED3_0	: 4;
		uint8_t RDSEL01_SG			: 1;
		uint8_t RDSEL01_OT			: 1;
		uint8_t RDSEL01_OTPW		: 1;
		uint8_t RDSEL01_SHORTA_S2GA	: 1;
	};

 } TMC262C_ReadBackRDSEL01_t;	

/*
 * @brief Device configuration read type register RDSEL10.
 *
 **/
 typedef union __attribute__((aligned(1), packed)) {

	struct {
		uint8_t ReadBack_MSB_H;
		uint8_t ReadBack_MSB_L;
		uint8_t ReadBack_LSB_L;
	};

	struct {
		/* RDSEL = 10. */
		uint8_t RDSEL10_SE4_2		: 3;
		uint8_t RDSEL10_SG9_5		: 5;

		uint8_t RDSEL10_SHORTB_S2GB	: 1;
		uint8_t RDSEL10_OLA			: 1;
		uint8_t RDSEL10_OLB			: 1;
		uint8_t RDSEL10_STST		: 1;
		/* Reads as 00. */
		uint8_t RDSEL10_RESERVED9_8	: 2;
		uint8_t RDSEL10_SE1_0		: 2;

		uint8_t RDSEL10_RESERVED3_0	: 4;
		uint8_t RDSEL10_SG			: 1;
		uint8_t RDSEL10_OT			: 1;
		uint8_t RDSEL10_OTPW		: 1;
		uint8_t RDSEL10_SHORTA_S2GA	: 1;
	};

 } TMC262C_ReadBackRDSEL10_t;

/*
 * @brief Device configuration read type register RDSEL11.
 *
 **/
 typedef union __attribute__((aligned(1), packed)) {

	struct {
		uint8_t ReadBack_MSB_H;
		uint8_t ReadBack_MSB_L;
		uint8_t ReadBack_LSB_L;
	};

	struct {
		/* RDSEL = 11. */
		uint8_t RDSEL11_OT136		: 1;
		uint8_t RDSEL11_OT150 		: 1;
		uint8_t RDSEL11_S2GA 		: 1;
		uint8_t RDSEL11_S2VSA 		: 1;
		uint8_t RDSEL11_S2GB 		: 1;
		uint8_t RDSEL11_S2VSB		: 1;
		uint8_t RDSEL11_ENN_IN		: 1;
		uint8_t RDSEL11_UV_7V		: 1;

		uint8_t RDSEL11_SHORTB_S2GB	: 1;
		uint8_t RDSEL11_OLA			: 1;
		uint8_t RDSEL11_OLB			: 1;
		uint8_t RDSEL11_STST		: 1;
		/* Reads as 11. */
		uint8_t RDSEL11_RESERVED9_8	: 2;
		uint8_t RDSEL11_OT100		: 1;
		uint8_t RDSEL11_OT120		: 1;

		uint8_t RDSEL11_RESERVED3_0	: 4;
		uint8_t RDSEL11_SG			: 1;
		uint8_t RDSEL11_OT			: 1;
		uint8_t RDSEL11_OTPW		: 1;
		uint8_t RDSEL11_SHORTA_S2GA	: 1;
	};

 } TMC262C_ReadBackRDSEL11_t;

/*
 * @brief Driver control register's set.
 *
 **/
typedef struct DrvCtrl_t {

		TMC262C_DrvCtrl_SDOFF1_t DrvCtrlSPI;
		TMC262C_DrvCtrl_SDOFF0_t DrvCtrlSTEPDIR;

	} TMC262C_DrvCtrlRSet_t;

/*
 * @brief Readback register's set.
 *
 **/
 typedef struct {

	TMC262C_ReadBackRDSEL00_t ReadBackRDSEL00;
	TMC262C_ReadBackRDSEL01_t ReadBackRDSEL01;
	TMC262C_ReadBackRDSEL10_t ReadBackRDSEL10;
	TMC262C_ReadBackRDSEL11_t ReadBackRDSEL11;

 } TMC262C_ReadBack_t;

/*
 * @brief Data exchange function typedefs.
 *
 **/
typedef void(*delay_fptr)(uint32_t delay);
typedef void(*spi_txrx_fptr)(uint8_t *pData, uint8_t size);

/*
 * @brief General data instance struct.
 *
 **/
typedef struct {

	delay_fptr delay;
	spi_txrx_fptr spi_tx;
	spi_txrx_fptr spi_rx;

} TMC262C_GInst_t;


/* Public function prototypes. */
void TMC262C_EnableCtrl(uint8_t state);
void TMC262C_Init(uint8_t motor_profile);
void TMC262C_DirectionCtrl(uint8_t direction);
void TMC262C_SetMotorProfile(uint8_t motor_profile);
void TMC262C_SetStepRes(uint8_t step_res);

TMC262C_SgcsConf_t TMC262C_StallGuard(uint8_t sfilt, uint8_t sgt_sign, uint8_t sgt, uint8_t cs, uint8_t read_back) ;
TMC262C_DrvCtrlRSet_t TMC262C_DrvCtrl(uint8_t sdoff_state, uint8_t intpol, uint8_t dedge, uint8_t mres, 
										uint8_t pha_polarity_a, uint8_t ca_current_a, uint8_t phb_polarity_b, uint8_t cb_current_b, uint8_t read_back);
TMC262C_ChopConf_t TMC262C_ChopConf(uint8_t tbl, uint8_t chm, uint8_t rndtf, uint8_t hdec1,
										uint8_t hdec0, uint8_t hend, uint8_t hstrt, uint8_t toff, uint8_t read_back);
TMC262C_SmartEn_t TMC262C_SmartEn(uint8_t semin, uint8_t sedn, uint8_t semax, uint8_t seup, 
									uint8_t semin3_0, uint8_t read_back);
TMC262C_DrvConf_t TMC262C_DrvConf(uint8_t tst, uint8_t slph, uint8_t slpl, uint8_t tmc262c_slp2, uint8_t dis_s2g, 
									uint8_t ts2g, uint8_t sdoff, uint8_t vsense, uint8_t rdsel, uint8_t tmc262c_otsens, 
									uint8_t tmc262c_shrtsens, uint8_t tmc262c_en_pfd, uint8_t tmc262c_en_s2vs, uint8_t read_back);
TMC262C_ReadBack_t* TMC262C_ReadBack(void);

void TMC262C_SetSpeed(uint32_t speed);

#endif /* TMC262C_H_ */








