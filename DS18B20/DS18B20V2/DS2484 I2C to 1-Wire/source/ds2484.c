/*
 *	@brief DS2484 communication functions.
 *	Created 03.08.21
 *
 **/

#include "stm32f1xx_hal.h"
#include "ds2484.h"

/*
 * @brief Private function prototypes.
 *
 **/
static void DS2484_SetReadPointer(DS2484_GInst_t *device, DS2484_PReadCode pcode);
static DS2484_StatusReg_t DS2484_ReadStatusReg(DS2484_GInst_t *device);
static uint8_t DS2484_1WireWaitBusy(DS2484_GInst_t *device);
static void DS2484_RxTxComplete(DS2484_GInst_t *device);


/* Public functions. */

/*
 * @brief Generates a 1-Wire reset/presence-detect cycle at the 1-Wire line. The state of
 * the 1-Wire line is sampled at tSI and tMSP and the result is reported to the host processor
 * through the Status register bits PPD and SD.
 * 
 * param *device : pointer to the instance of the DS2484_GInst_t.
 * return : If presence occured, return 1, zero if not.
 * 
 **/
uint8_t DS2484_1WireReset(DS2484_GInst_t *device)
{	
	DS2484_StatusReg_t status_reg = { 0 };
	
	device->data_struct.Command = DS2484_1W_RESET;
	device->i2c_tx_data(DS2484_ADDR_SHIFTED, &device->data_struct.Command, 1);	
	DS2484_RxTxComplete(device);
	/* Wait not less than 480uS. */
	if (DS2484_1WireWaitBusy(device) == DS2484_ErrTimeout) return 0;
	status_reg = DS2484_ReadStatusReg(device);		
	/* Read presence detect. */	
	return status_reg.PPD;
} 

/*
 * @brief Transmit data to the 1-Wire bus.
 * 
 * param *device : pointer to the instance of the DS2484_GInst_t.
 * return status : 
 * 
 **/
uint8_t DS2484_1WireWriteData(DS2484_GInst_t *device, uint8_t* data, uint8_t size)
{	
	uint8_t status = 0;
	
	for (uint8_t i = size; i > 0; i--) {
		status = DS2484_1WireWriteByte(device, *data);
		data++;
	}
	return status;
} 

/*
 * @brief Receive data from the 1-Wire bus.
 * 
 * param *device : pointer to the instance of the DS2484_GInst_t.
 * 
 **/
void DS2484_1WireReadData(DS2484_GInst_t *device, uint8_t* data, uint8_t size)
{	
	for (uint8_t i = size; i > 0; i--) {
		*data = DS2484_1WireReadByte(device);
		data++;
	}
} 

/*
 * @brief Function for enable or disable strong pull-up on the one wire bus.
 * 
 * param state : if 1 - the strong pull-up is enable, 0 is disable. See the DS2484_CFGReg_t typedef for more detaled.
 *
 **/
void DS2484_1WireSPU(DS2484_GInst_t *device, uint8_t state)
{
	DS2484_CFGReg_t cfg_byte;
	
	DS2484_SetReadPointer(device, DS2484_CFG_PREG);
	device->i2c_rx_data(DS2484_ADDR_SHIFTED, &cfg_byte.CFGReg, 1);	
	DS2484_RxTxComplete(device);
	cfg_byte.SPU = state;
	DS2484_WriteDeviceConfiguration(device, cfg_byte.APU, cfg_byte.PDN, cfg_byte.SPU, cfg_byte.ONEWS);
}

/*
 * @brief
 * 
 * param state : if 1 - the active pull-up is enable, 0 is disable. See the DS2484_CFGReg_t typedef for more detaled.
 *
 **/
void DS2484_1WireAPU(DS2484_GInst_t *device, uint8_t state)
{
	DS2484_CFGReg_t cfg_byte;
	
	DS2484_SetReadPointer(device, DS2484_CFG_PREG);
	device->i2c_rx_data(DS2484_ADDR_SHIFTED, &cfg_byte.CFGReg, 1);	
	DS2484_RxTxComplete(device);
	cfg_byte.APU = state;
	DS2484_WriteDeviceConfiguration(device, cfg_byte.APU, cfg_byte.PDN, cfg_byte.SPU, cfg_byte.ONEWS);
}

/*
 * @brief Generates a single 1-Wire time slot with a bit value as specified by the bit byte at the
 * 1-Wire line. A value of 0b generates a write-zero time slot; a value of
 * 1b generates a write-one time slot, which also functions as a read-data time slot. In
 * either case, the logic level at the 1-Wire line is tested at tMSR and SBR is updated.
 * 
 * param *device : pointer to the instance of the DS2484_GInst_t.
 * param time_slot_value : 0 is the write zero time slot, 1 is the write/read time slot.
 * 
 **/
uint8_t DS2484_1WireSingleBit(DS2484_GInst_t *device, uint8_t time_slot_value)
{		
	device->data_struct.Command = DS2484_1W_SINGLE_BIT;
	if (time_slot_value) device->data_struct.TxDataByte[0] = 0x80;
	else device->data_struct.TxDataByte[0] = 0x00;
	
	device->i2c_tx_data(DS2484_ADDR_SHIFTED, &device->data_struct.Command, 2);	
	DS2484_RxTxComplete(device);
	/* Wait until all 1-wire communications end.*/
	return DS2484_1WireWaitBusy(device);
} 

/*
 * @brief Writes a single data byte to the 1-Wire line.
 * To write commands or data to the 1-Wire line. Equivalent to executing eight 1-Wire Single Bit
 * commands, but faster due to less I2C traffic.
 * 
 * param *device : pointer to the instance of the DS2484_GInst_t.
 * param byte : commands or data to write to the 1-wire data bus.
 * 
 **/
uint8_t DS2484_1WireWriteByte(DS2484_GInst_t *device, uint8_t byte)
{		
	device->data_struct.Command = DS2484_1W_WRITE_BYTE;
	device->data_struct.TxDataByte[0] = byte;
	device->i2c_tx_data(DS2484_ADDR_SHIFTED, &device->data_struct.Command, 2);	
	DS2484_RxTxComplete(device);	
	/* Wait until all 1-wire communications end.*/
	return DS2484_1WireWaitBusy(device);
}

/*
 * @brief Generates eight read-data time slots on the 1-Wire line and stores result in the Read Data
 * register. To read data from the 1-Wire line. Equivalent to executing eight 1-Wire Single Bit commands
 * with value = 1 (write-one time slot), but faster due to less I2C traffic.
 * 
 * param *device : pointer to the instance of the DS2484_GInst_t.
 * 
 **/
uint8_t DS2484_1WireReadByte(DS2484_GInst_t *device)
{		
	device->data_struct.Command = DS2484_1W_READ_BYTE;
	device->i2c_tx_data(DS2484_ADDR_SHIFTED, &device->data_struct.Command, 1);	
	DS2484_RxTxComplete(device);	
	/* Wait until all 1-wire communications end.*/
	DS2484_1WireWaitBusy(device);
	/* Set pointer REG to the data byte. */
	DS2484_SetReadPointer(device, DS2484_READ_DATA_PREG); 
	device->i2c_rx_data(DS2484_ADDR_SHIFTED, device->data_struct.RxDataByte, 1);
	DS2484_RxTxComplete(device);
	return device->data_struct.RxDataByte[0];
}

/*
 * @brief Performs a global reset of device state machine logic. Terminates any ongoing 1-Wire
 * communication.
 * 
 * param *device : pointer to the instance of the DS2484_GInst_t.
 *
 **/
void DS2484_DeviceReset(DS2484_GInst_t *device)
{		
	device->data_struct.Command = DS2484_DEVICE_RESET;
	device->i2c_tx_data(DS2484_ADDR_SHIFTED, &device->data_struct.Command, 1);	
	DS2484_RxTxComplete(device);		
} 

/*
 * @brief Writes a new device configuration byte. The new settings take effect immediately. Note:
 * When writing to the Device Configuration register, the new data is accepted only if the upper
 * nibble (bits 7 to 4) is the one’s complement of the lower nibble (bits 3 to 0). When read, the
 * upper nibble is always 0h. See the DS2484_CFGReg_t in the header file.
 * 
 * param *device : pointer to the instance of the DS2484_GInst_t.
 * 
 **/
void DS2484_WriteDeviceConfiguration(DS2484_GInst_t *device, uint8_t apu, uint8_t pdn, uint8_t spu, uint8_t onews)
{		
	DS2484_CFGReg_t cfg_byte = { 0 };	
	
	cfg_byte.APU = apu;
	cfg_byte.PDN = pdn;
	cfg_byte.SPU = spu;
	cfg_byte.ONEWS = onews;
	cfg_byte.ONES_COMPLEMENT = ~cfg_byte.LOWER_NIBBLE;	
	
	device->data_struct.Command = DS2484_WRITE_DEVICE_CONFIGURATION;
	device->data_struct.TxDataByte[0] = cfg_byte.CFGReg;
	device->i2c_tx_data(DS2484_ADDR_SHIFTED, &device->data_struct.Command, 2);	
	DS2484_RxTxComplete(device);		
} 

/*
 * @brief Updates the selected 1-Wire port parameter, which affects the 1-Wire timing or pullup
 * resistor selection. Note: Upon a power-on reset or after a Device Reset command, the parameter 
 * default values apply. See the DS2484_ControlByte_t for the detaled information.
 * 
 * param *device : pointer to the instance of the DS2484_GInst_t.
 * 
 **/
void DS2484_1WirePortAdjust(DS2484_GInst_t *device, uint8_t val, uint8_t od, uint8_t p)
{		
	DS2484_ControlByte_t control_byte;
	
	control_byte.VAL_3_0 = val;
	control_byte.OD_4 = od;
	control_byte.P_7_5 = p;
	
	device->data_struct.Command = DS2484_1W_PORT_ADJUST;
	device->data_struct.TxDataByte[0] = control_byte.ControlByte;
	device->i2c_tx_data(DS2484_ADDR_SHIFTED, &device->data_struct.Command, 2);	
	DS2484_RxTxComplete(device);		
} 



/* Private functions. */

/*
 * @brief Obtain the status register content to perform check out its flags.
 * 
 **/
static uint8_t DS2484_1WireWaitBusy(DS2484_GInst_t *device)
{		
	uint32_t cntTimeOut = 50000;
	DS2484_StatusReg_t status_reg = { 0 };	

	do {
		cntTimeOut--;
		if (cntTimeOut == 0) return DS2484_ErrTimeout;
		device->i2c_rx_data(DS2484_ADDR_SHIFTED, &status_reg.StatusReg, 1);
		DS2484_RxTxComplete(device);
		
	} while (status_reg.ONEWB);
	return DS2484_NoErrTimeout;
}

/*
 * @brief Obtain the status register content to perform check out its flags.
 * 
 **/
static DS2484_StatusReg_t DS2484_ReadStatusReg(DS2484_GInst_t *device)
{		
	DS2484_StatusReg_t status_reg = { 0 };
	
	device->i2c_rx_data(DS2484_ADDR_SHIFTED, &status_reg.StatusReg, 1);
	DS2484_RxTxComplete(device);
	return status_reg;
} 

/*
 * @brief Sets the read pointer to the specified register. Overwrites the read pointer position of any
 * 1-Wire communication command in progress.
 * 
 **/
static void DS2484_SetReadPointer(DS2484_GInst_t *device, DS2484_PReadCode pcode)
{		
	device->data_struct.Command = DS2484_SET_READ_POINTER;
	device->data_struct.TxDataByte[0] = pcode;
	device->i2c_tx_data(DS2484_ADDR_SHIFTED, &device->data_struct.Command, 2);	
	DS2484_RxTxComplete(device);		
} 

/*
 * @brief Waiting reception/transmition complete on the I2C bus.
 *
 **/
static void DS2484_RxTxComplete(DS2484_GInst_t *device)
{
	/* About 1400nS for 72MHz(t = 13.8nS). */
	uint32_t timeout = 100;
	
	while (*device->isReceiveComplete > 0)
	{
		///Wait a transmition complete.	
		timeout--;
		if (!timeout)
		{
			break;
		}
	}
}