/*
 * @brief Common.c
 *
 * Created on: Jul 16, 2023
 * Author: asw3005
 *
 */
#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "main.h"
#include "Common.h"
#include "DataExchCAN.h"
#include "CRCCalc.h"
#include "string.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

/* External variables. */
extern I2C_HandleTypeDef hi2c1;
extern QueueHandle_t CanDataTx_qh;
extern SemaphoreHandle_t SoftBtnPressed_sh;

/* Private variables. */
static I2C_HandleTypeDef* ExtMemI2C = &hi2c1;

static M24128_MEM_DATA_t UserData = {
	.BYTE_ADDR = GEN_DATA_BYTE,
	.PAGE_ADDR = GEN_DATA_PAGE
};

/*
 * @brief Get command codes.
 */


/*
 * @brief Make command code for each of the text command.
 */
uint32_t* MCS_MakeCmdCodes(void) {

	//uint32_t size;
	/* List of commands. */
	static char* const CmdList[] = {
										/* Hardware reading request. */
										"MCS_HardwareReq",
										/* Command code to connect to the PCB. */
										"MCS_ConnectTo",
										/* MCS command list. */
										"MCS_GETSysInfo",
										"MCS_GETDeviceList",
										"MCS_SETEthAddr",
										"MCS_SETAddr",
										"MCS_ENBip",
										"MCS_DISBip",
										"MCS_GetParams",
										"MCS_SetParams",
										"MCS_SaveParams",
										"MCS_RestoreParams",
										"MCS_SyncDevices",
										/* FPGA FIFO command list. */
										"FPGA_FifoReset",
										"FPGA_GetSampleNumbers",
										"FPGA_GetFifo0samples",
										"FPGA_GetFifo1samples",
										"FPGA_GetTSampleNumbers",
										"FPGA_GetTSamples",
										/* Configuration commands. */
										"FPGA_SoftwareStart",
										"FPGA_SyncRegisters",
										"FPGA_SetDefaulParams",
										"FPGA_SetCtrlReg",
										"FPGA_SetPeriodLimiter",
										"FPGA_SetFStartTim",
										"FPGA_SetSStartTim",
										"FPGA_SetFStopTim",
										"FPGA_SetSStopTim",
										"FPGA_SetThrStartHigh",
										"FPGA_SetThrStartLow",
										"FPGA_SetThr0Stop",
										"FPGA_SetThr1Stop",
										/* Global command set. */
										"GCS_EnDisDevice",
										"GCS_VIEWAddr",
										/* Broadcast command set. */
										"BCS_EnDisDevice",
										"BCS_VIEWAddr",
										/* Unicast command set. */
										"UCS_GETInfo",
										"UCS_EnDisDevice",
										"UCS_SETAddr",
										"UCS_LOCKAddr",
										/* Command prefixes. */
										"FPGA",
										"MCS",
										"GCS",
										"BCS",
										"UCS",
										"AGU"
									};

	static uint32_t CommandCode[sizeof(CmdList)/sizeof(char*)];

	//size = sizeof(CmdList);
	//size = sizeof(CmdList)/sizeof(char*);
	//size = strlen(CmdList[0]);

	for(uint8_t i = 0; i < sizeof(CmdList)/sizeof(char*); i++) {
		CommandCode[i] = Hard_CRC32ETH((uint8_t*)CmdList[i], strlen(CmdList[i]));
	}
	__NOP();

	return CommandCode;
}

/*
 * @brief Read all generators' data from external memory on the board.
 *
 * @param ReadWrite : 0 to write, 1 to read.
 */
void DEV_RWGenEepAll(uint8_t ReadWrite) {

	UserData.BYTE_ADDR = GEN_DATA_BYTE;
	UserData.PAGE_ADDR = GEN_DATA_PAGE;
	if (!ReadWrite) {
	/* Read 48 bytes of generators' data per channel. */
//	for (uint8_t i = 0; i < MAX_SGU_CHANNELS; i++) {
//		UserData.PAGE_ADDR = GEN_DATA_PAGE + i*3;
//		HAL_I2C_Mem_Read(ExtMemI2C, EEP_DATA_ADDR_SHIFTED, UserData.MemAddr, I2C_MEMADD_SIZE_16BIT, (uint8_t*)&DEV_GetGenData()[i].Phase[0],
//			sizeof(DEV_GenData_t)*MAX_UPCB_NUMBER, 25);
//		UserData.PAGE_ADDR = GEN_DATA_PAGE + 1 + i*3;
//		HAL_I2C_Mem_Read(ExtMemI2C, EEP_DATA_ADDR_SHIFTED, UserData.MemAddr, I2C_MEMADD_SIZE_16BIT, (uint8_t*)&DEV_GetGenData()[i].Amplitude[0],
//			sizeof(DEV_GenData_t)*MAX_UPCB_NUMBER, 25);
//		UserData.PAGE_ADDR = GEN_DATA_PAGE + 2 + i*3;
//		HAL_I2C_Mem_Read(ExtMemI2C, EEP_DATA_ADDR_SHIFTED, UserData.MemAddr, I2C_MEMADD_SIZE_16BIT, (uint8_t*)&DEV_GetGenData()[i].Frequency[0],
//			sizeof(DEV_GenData_t)*MAX_UPCB_NUMBER, 25);
//		}
//
//	} else {
//		/* Write 48 bytes of generators' data per channel. */
//		for (uint8_t i = 0; i < MAX_SGU_CHANNELS; i++) {
//			UserData.PAGE_ADDR = GEN_DATA_PAGE + i*3;
//			HAL_I2C_Mem_Write(ExtMemI2C, EEP_DATA_ADDR_SHIFTED, UserData.MemAddr, I2C_MEMADD_SIZE_16BIT, (uint8_t*)&DEV_GetGenData()[i].Phase[0],
//				sizeof(DEV_GenData_t)*MAX_UPCB_NUMBER, 25);
//			UserData.PAGE_ADDR = GEN_DATA_PAGE + 1 + i*3;
//			HAL_I2C_Mem_Write(ExtMemI2C, EEP_DATA_ADDR_SHIFTED, UserData.MemAddr, I2C_MEMADD_SIZE_16BIT, (uint8_t*)&DEV_GetGenData()[i].Amplitude[0],
//				sizeof(DEV_GenData_t)*MAX_UPCB_NUMBER, 25);
//			UserData.PAGE_ADDR = GEN_DATA_PAGE + 2 + i*3;
//			HAL_I2C_Mem_Write(ExtMemI2C, EEP_DATA_ADDR_SHIFTED, UserData.MemAddr, I2C_MEMADD_SIZE_16BIT, (uint8_t*)&DEV_GetGenData()[i].Frequency[0],
//				sizeof(DEV_GenData_t)*MAX_UPCB_NUMBER, 25);
//		}
	}
}

/*
 * @brief Getter for DEV_TypeList_t struct.
 */
DEV_List_t* DEV_GetType(void) {

	static DEV_List_t DevList = { 0 };
	return &DevList;
}

/*
 * @brief Device available list.
 */
void DEV_WriteDevType(uint8_t dev_channel, uint32_t dev_type, uint8_t dev_subtype) {

	if ((dev_channel) <= MAX_UPCB_NUMBER) {
		DEV_GetType()->DevType[dev_channel - 1] = dev_type;
		DEV_GetType()->DevSubType[dev_channel - 1] = dev_subtype;
	}
}

/*
 * @brief Update the list of available device .
 */
void DEV_UpdateState(uint32_t dev_type, uint32_t sprefix, uint8_t address) {

	CAN_TxQueueData_t CAN_TxData = { 0 };

	CAN_TxData.DevType = dev_type;
	CAN_TxData.SPrefixCode = sprefix;

	/* Clear the list of devices. */
	HAL_Delay(100);
	DEV_WriteDevType(address, NO_DEVICE_AVAILABLE, NO_DEVICE_AVAILABLE);
	CAN_TxData.Address = address;
	CAN_TxData.Command = UCS_DEV_LINK;
	xQueueSendToBack(CanDataTx_qh, &CAN_TxData, 0);
	HAL_Delay(75);
}

/*
 * @brief Update the list of available devices.
 */
void DEV_UpdateList() {

	CAN_TxQueueData_t CAN_TxData = { 0 };

	CAN_TxData.DevType		= TYPE_UNDEFINED;
	CAN_TxData.SPrefixCode	= UCS;

	/* Waiting module available. */
	HAL_Delay(50);

	/* Clear list of available devices. */
	for (uint8_t i = 1; i <= MAX_UPCB_NUMBER; i++) {
		DEV_WriteDevType(i, NO_DEVICE_AVAILABLE, NO_DEVICE_AVAILABLE);
	}

	/*  */
	for (uint8_t i = 1; i <= MAX_UPCB_NUMBER; i++) {
		CAN_TxData.Address = UCS1_CMD_BASE_OFFSET + i - 1;
		CAN_TxData.Command = UCS_DEV_LINK;
		xQueueSendToBack(CanDataTx_qh, &CAN_TxData, 0);
		HAL_Delay(75);
	}
}

/*
 * @brief Update devices' parameters.
 */
void DEV_UpdateParams() {

	CAN_TxQueueData_t CAN_TxData = { 0 };

	CAN_TxData.DevType		= AGU;
	CAN_TxData.SPrefixCode	= UCS;

	/* Waiting module available. */
	HAL_Delay(50);

	/* Set phase to all available devices. */
//	CAN_TxData.CmdCode		= BCS_SETAmplitude;
//	for (uint8_t i = 1; i <= MAX_UPCB_NUMBER; i++) {
//		if (DEV_GetType()->DevType[i - 1] == SGU) {
//			CAN_TxData.Address = UCS1_CMD_BASE_OFFSET + i - 1;
//			CAN_TxData.Command = UCS_SET_AMPLITUDE;
//			for(uint8_t j = 0; j < MAX_SGU_CHANNELS; j++) {
//				CAN_TxData.Phase = DEV_GetGenData()[i - 1].Amplitude[j];
//				xQueueSendToBack(CanDataTx_qh, &CAN_TxData, 0);
//				HAL_Delay(75);
//			}
//		}
//	}
}

/*
 * @brief Device name decoder.
 **/
void DEV_NameDecode(uint8_t State, uint32_t DevType, uint8_t DevSubtype, DEV_CharBuff_t* CharBuff) {

	/* State decoding to char. */
	switch (State) {
		case DEV_OFF:
			sprintf(&CharBuff->state[0], "OFF");
			break;
		case DEV_ON:
			sprintf(&CharBuff->state[0], "ON");
			break;
		default:
			sprintf(&CharBuff->state[0], "N.A.");
			break;
	}

	/* Select the right type of addressed device. */
	switch (DevType) {
		case AGU:
			sprintf(&CharBuff->dev_id[0], "AGU");
			break;
		default:
			sprintf(&CharBuff->dev_id[0], "N.A.");
			break;
	}

	/* Subtype decoding to char. */
	switch (DevSubtype) {
		case AGU0:
			sprintf(&CharBuff->dev_subid[0], "AGU0");
			break;
		default:
			sprintf(&CharBuff->dev_subid[0], "N.A.");
			break;
	}
}

/*
 * @brief Convert decimal to state binary byte.
 */
uint8_t DEV_DecToStateByte(uint32_t DecValue) {

	/* . */
	uint8_t StateByte = 0;

	/* Disable all if value is bigger than. */
	if (DecValue > 99999999) {
		return 0;
	}
	/* Conversion if it's ok. */
	for (uint8_t i = 7; i > 0; i--) {
		if ((DecValue % 10) > 0) {
			StateByte |= 0x80;
		}
		DecValue /= 10;
		StateByte >>= 1;
	}
	if (DecValue  > 0) {
		StateByte |= 0x80;
	}
	return StateByte;
}

/*
 * @brief Led channel control.
 *
 * @param channel: can have there values -	CH1, CH2, CH3, CH4, CH5, CH6, CH_ALL
 * @param state: If state equal to 0, led is power off. If state above 0, led is power on.
 *
 */
void LED_Ctrl(uint8_t channel, uint8_t state) {

//	switch (channel) {
//
//	case CH1:
//		if (state) { HAL_GPIO_WritePin(LED1_CH1_GPIO_Port, LED1_CH1_Pin, GPIO_PIN_RESET); }
//		else { HAL_GPIO_WritePin(LED1_CH1_GPIO_Port, LED1_CH1_Pin, GPIO_PIN_SET); }
//		break;
//	case CH2:
//		if (state) { HAL_GPIO_WritePin(LED2_CH2_GPIO_Port, LED2_CH2_Pin, GPIO_PIN_RESET); }
//		else { HAL_GPIO_WritePin(LED2_CH2_GPIO_Port, LED2_CH2_Pin, GPIO_PIN_SET); }
//		break;
//	case CH3:
//		if (state) { HAL_GPIO_WritePin(LED3_CH3_GPIO_Port, LED3_CH3_Pin, GPIO_PIN_RESET); }
//		else { HAL_GPIO_WritePin(LED3_CH3_GPIO_Port, LED3_CH3_Pin, GPIO_PIN_SET); }
//		break;
//	case CH4:
//		if (state) { HAL_GPIO_WritePin(LED4_CH4_GPIO_Port, LED4_CH4_Pin, GPIO_PIN_RESET); }
//		else { HAL_GPIO_WritePin(LED4_CH4_GPIO_Port, LED4_CH4_Pin, GPIO_PIN_SET); }
//		break;
//	case CH5:
//		if (state) { HAL_GPIO_WritePin(LED5_CH5_GPIO_Port, LED5_CH5_Pin, GPIO_PIN_RESET); }
//		else { HAL_GPIO_WritePin(LED5_CH5_GPIO_Port, LED5_CH5_Pin, GPIO_PIN_SET); }
//		break;
//	case CH6:
//		if (state) { HAL_GPIO_WritePin(LED6_CH6_GPIO_Port, LED6_CH6_Pin, GPIO_PIN_RESET); }
//		else { HAL_GPIO_WritePin(LED6_CH6_GPIO_Port, LED6_CH6_Pin, GPIO_PIN_SET); }
//		break;
//	case CH_ALL:
//		if (state) {
//			HAL_GPIO_WritePin(LED1_CH1_GPIO_Port, LED1_CH1_Pin, GPIO_PIN_RESET);
//			HAL_GPIO_WritePin(LED2_CH2_GPIO_Port, LED2_CH2_Pin, GPIO_PIN_RESET);
//			HAL_GPIO_WritePin(LED3_CH3_GPIO_Port, LED3_CH3_Pin, GPIO_PIN_RESET);
//			HAL_GPIO_WritePin(LED4_CH4_GPIO_Port, LED4_CH4_Pin, GPIO_PIN_RESET);
//			HAL_GPIO_WritePin(LED5_CH5_GPIO_Port, LED5_CH5_Pin, GPIO_PIN_RESET);
//			HAL_GPIO_WritePin(LED6_CH6_GPIO_Port, LED6_CH6_Pin, GPIO_PIN_RESET);
//		} else {
//			HAL_GPIO_WritePin(LED1_CH1_GPIO_Port, LED1_CH1_Pin, GPIO_PIN_SET);
//			HAL_GPIO_WritePin(LED2_CH2_GPIO_Port, LED2_CH2_Pin, GPIO_PIN_SET);
//			HAL_GPIO_WritePin(LED3_CH3_GPIO_Port, LED3_CH3_Pin, GPIO_PIN_SET);
//			HAL_GPIO_WritePin(LED4_CH4_GPIO_Port, LED4_CH4_Pin, GPIO_PIN_SET);
//			HAL_GPIO_WritePin(LED5_CH5_GPIO_Port, LED5_CH5_Pin, GPIO_PIN_SET);
//			HAL_GPIO_WritePin(LED6_CH6_GPIO_Port, LED6_CH6_Pin, GPIO_PIN_SET);
//		}
//		break;
//	default:
//		break;
//	}

}

/*
 * @brief Sync pulse issue.
 */
void DEV_GiveSoftSyncPulse(void) {

//	DEV_SyncInSel(1);
//	HAL_GPIO_WritePin(MCU_SYNC_PULSE_GPIO_Port, MCU_SYNC_PULSE_Pin, GPIO_PIN_SET);
//	HAL_Delay(1);
//	HAL_GPIO_WritePin(MCU_SYNC_PULSE_GPIO_Port, MCU_SYNC_PULSE_Pin, GPIO_PIN_RESET);
//	HAL_Delay(1);
//	DEV_SyncInSel(0);

}

/*
 * @brief Software button trigger.
 */
void BTN_SWPress(void) {

	/* Giving the semaphore. */
	xSemaphoreGive(SoftBtnPressed_sh);
	/* Software triggered external interrupt input. It's like press a button on the pin 3. */
	EXTI->SWIER = EXTI_SWIER_SWIER3;
}
