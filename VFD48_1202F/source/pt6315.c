/*
 *	@brief PT6315 communication functions.
 *	Created 04.18.21
 *
 **/

#include "stm32f1xx_hal.h"
#include "pt6315.h"

/* Private variables. */

/* RAM memory of PT6315. */
static struct __attribute__((aligned(1), packed)) ram_data_t 
{
	PT6315_AddressSetCmd_t cmd;
	uint8_t pt6315_ram[12][3];
	
} ram_data;


/* Private function prototypes. */
//static void PT6315_WriteRAM(PT6315_GInst_t device);
static void PT6315_SendModeCmd(PT6315_GInst_t *device, PT6315_DisplayModeCmd mode);
static void PT6315_SendControlCmd(PT6315_GInst_t *device, PT6315_DisplayControlCmd display_onoff, PT6315_DisplayControlCmd dimming);
static void PT6315_SendDataSetCmd(PT6315_GInst_t *device,
	PT6315_DataSetCmd mode,
	PT6315_DataSetCmd addr_mode, 
	PT6315_DataSetCmd data_mode);
static void PT6315_SetAddress(PT6315_GInst_t *device, uint8_t address);



/* Public functions. */

/*
 * @brief PT6315 init.
 *
 **/
void PT6315_Init(PT6315_GInst_t *device)
{
	PT6315_SendModeCmd(device, PT6315_12DIG_16SEG);
	PT6315_SendControlCmd(device, PT6315_DISPLAY_ON, PT6315_PULSE_WIDTH_1_16);
	
	for (uint8_t i = 0; i < 12; i++) {
		for (uint8_t j = 0; j < 3; j++) {
			ram_data.pt6315_ram[i][j] = 0x00;
		}
	}
	
	PT6315_WriteRAM(device);
}

/*
 * @brief Set or reset required data bit of the RAM memory of the PT6315.
 * 
 * @param device : instance of general data struct. 
 * @param segment : number of segment of the PT6315 driver (from 1 to 24).
 * @param dig : number of dig of the PT6315 driver (from 1 to 12).
 * @param state : setting or resetting specific segment (1 is set, 0 is reset).
 *
 **/
void PT6315_SetResetRAMBuff(PT6315_GInst_t *device, uint8_t segment, uint8_t dig, uint8_t state)
{	
	if (dig < PT6315_MIN_NUMBER_OF_DIG || dig > PT6315_MAX_NUMBER_OF_DIG || 
		segment < PT6315_MIN_NUMBER_OF_SEG || segment > PT6315_MAX_NUMBER_OF_SEG) return;
	
	/* If required segment from 1 to 8. */
	if (segment <= 8) {
		/* Set or reset required bit in the RAM memory. */
		if (state) {
			ram_data.pt6315_ram[dig - 1][0] |= (0x01 << (segment - 1));
		} else {
			ram_data.pt6315_ram[dig - 1][0] &= ~(0x01 << (segment - 1));
		}	
	/* If required segment from 9 to 16. */
	} else if (segment < 17) {
		if (state) {
			ram_data.pt6315_ram[dig - 1][1] |= (0x01 << (segment - 9));
		}
		else {
			ram_data.pt6315_ram[dig - 1][1] &= ~(0x01 << (segment - 9));
		}
	/* If required segment from 17 to 24. */
	}  else {
		if (state) {
			ram_data.pt6315_ram[dig - 1][2] |= (0x01 << (segment - 17));
		}
		else {
			ram_data.pt6315_ram[dig - 1][2] &= ~(0x01 << (segment - 17));
		}
	}		
}

/*
 * @brief Set or reset required data bit of the RAM memory of the PT6315.
 * 
 * @param device : instance of general data struct. 
 * @param segment : number of segment of the PT6315 driver (from 1 to 24).
 * @param dig : number of dig of the PT6315 driver (from 1 to 12).
 *
 **/
uint8_t PT6315_ReadRAMBuff(PT6315_GInst_t *device, uint8_t segment, uint8_t dig)
{	
	if (dig < PT6315_MIN_NUMBER_OF_DIG || dig > PT6315_MAX_NUMBER_OF_DIG || 
		segment < PT6315_MIN_NUMBER_OF_SEG || segment > PT6315_MAX_NUMBER_OF_SEG) return 0xFF;
	
	/* If required segment from 1 to 8. */
	if (segment <= 8) {
		/* Reads required bit in the RAM memory. */
		return (ram_data.pt6315_ram[dig - 1][0] & (0x01 << (segment - 1))) >> (segment - 1);
	/* If required segment from 9 to 16. */
	} else if (segment < 17) {
		return (ram_data.pt6315_ram[dig - 1][1] & (0x01 << (segment - 9))) >> (segment - 9);
	/* If required segment from 17 to 24. */
	} else {
		return (ram_data.pt6315_ram[dig - 1][2] & (0x01 << (segment - 17))) >> (segment - 17);
	}		
}

/*
 * @brief Reads the keys connected to the PT6315.
 * 
 * @param device : instance of general data struct. 
 *
 **/
void PT6315_ReadKeys(PT6315_GInst_t *device)
{
	PT6315_DataSetCmd_t data_set_cmd;
	
	data_set_cmd.CMD = PT6315_DATA_CMD;
	data_set_cmd.SETTINGS_MODE = PT6315_NORMAL_MODE;
	data_set_cmd.ADDR_MODE = PT6315_INC_ADDR;
	data_set_cmd.DATA_RW_MODE = PT6315_READ_KEY;
	
	device->latch_control_fptr(0);	
	device->spi_tx_data_fptr(&data_set_cmd.DataSettingCmd, 1);	
	device->spi_rx_data_fptr((uint8_t *)&device->keys, 4);	
	//device->delay_fptr(1);
	device->latch_control_fptr(1);
}

/*
 * @brief Enables/disables specific LED.
 * 
 * @param device : instance of general data struct. 
 * @param number : number of LED. It can be from 1 to 4. All other values are ignored.
 * @param state : one is enable LED, zero is disable.
 *
 **/
void PT6315_LedEn(PT6315_GInst_t *device, uint8_t number, uint8_t state)
{
	/* LEDs state byte. */
	static struct tx
	{
		PT6315_DataSetCmd_t data_set_cmd;
		PT6315_LedControlCmd_t pt6315_ledbyte;
		
	} data;

	data.data_set_cmd.CMD = PT6315_DATA_CMD;
	data.data_set_cmd.SETTINGS_MODE = PT6315_NORMAL_MODE;
	data.data_set_cmd.ADDR_MODE = PT6315_INC_ADDR;
	data.data_set_cmd.DATA_RW_MODE = PT6315_WRITE_LEDPORT;
	
	if (number == 1) {
		data.pt6315_ledbyte.LED_1 = state; 
	} else if (number == 2) {
		data.pt6315_ledbyte.LED_2 = state; 
	} else if (number == 3) {
		data.pt6315_ledbyte.LED_3 = state; 
	} else if (number == 4) {
		data.pt6315_ledbyte.LED_4 = state; 
	} else {
		return;
	}		
	
	device->latch_control_fptr(0);
	device->spi_tx_data_fptr(&data.data_set_cmd.DataSettingCmd, 2);			
	device->latch_control_fptr(1);
	device->delay_fptr(1);
}

/*
 * @brief Enables/disables specific LED.
 * 
 * @param device : instance of general data struct. 
 * @param level : brightness value from the PT6315_DisplayControlCmd enum.
 *
 **/
void PT6315_SetBrightness(PT6315_GInst_t *device, PT6315_DisplayControlCmd level)
{
	/* Set the brightnees that you need. */
	PT6315_SendControlCmd(device, PT6315_DISPLAY_ON, level);
}


/*
 * @brief Writes the RAM memory of the PT6315.
 * 
 * @param device : instance of general data struct. 
 *
 **/
void PT6315_WriteRAM(PT6315_GInst_t *device)
{	
	
	ram_data.cmd.CMD = PT6315_ADDR_CMD;
	ram_data.cmd.ADDR = 0;	 
	
	PT6315_SendDataSetCmd(device, PT6315_NORMAL_MODE, PT6315_INC_ADDR, PT6315_WRITE_DISPLAY);
	
	device->latch_control_fptr(0);
	device->spi_tx_data_fptr((uint8_t*)&ram_data.cmd, sizeof(ram_data));	
	device->latch_control_fptr(1);
	device->delay_fptr(1);
}



/* Private functions. */


/*
 * @brief Sending display mode setting command.
 * 
 * @param device : instance of general data struct. 
 * @param mode : determine the number of segment and grids to be used, see the PT6315_DisplayModeCmd.
 *
 **/
static void PT6315_SendModeCmd(PT6315_GInst_t *device, PT6315_DisplayModeCmd mode)
{
	PT6315_DisplayModeCmd_t mode_cmd;
	
	mode_cmd.CMD = PT6315_DISP_CMD;
	mode_cmd.DisplaySettingCmd = mode;
	
	device->latch_control_fptr(0);
	device->spi_tx_data_fptr(&mode_cmd.DisplaySettingCmd, 1);
	
	device->latch_control_fptr(1);
	device->delay_fptr(1);
}


/*
 * @brief Sending display control command.
 * 
 * @param device : instance of general data struct. 
 * @param display_onoff : used to turn on or off a display, see the PT6315_DisplayControlCmd.
 * @param dimming : set the pulse width, see the PT6315_DisplayControlCmd.
 *
 **/
static void PT6315_SendControlCmd(PT6315_GInst_t *device, PT6315_DisplayControlCmd display_onoff, PT6315_DisplayControlCmd dimming)
{
	PT6315_DisplayControlCmd_t control_cmd;
	
	control_cmd.CMD = PT6315_CONTROL_CMD;
	control_cmd.DISPLAY_ON_OFF = display_onoff;
	control_cmd.PULSE_WIDTH = dimming;
	
	device->latch_control_fptr(0);
	device->spi_tx_data_fptr(&control_cmd.DisplayControlCmd, 1);
	device->delay_fptr(1);
	device->latch_control_fptr(1);
}


/*
 * @brief Sending data setting command.
 * 
 * @param device : instance of general data struct. 
 * @param mode : set operation mode of the display, see the PT6315_DataSetCmd.
 * @param addr_mode : set address operation mode, see the PT6315_DataSetCmd.
 * @param data_mode : select the data operatin mode, see the PT6315_DataSetCmd.
 *
 **/
static void PT6315_SendDataSetCmd(PT6315_GInst_t *device,
	PT6315_DataSetCmd mode,
	PT6315_DataSetCmd addr_mode, 
	PT6315_DataSetCmd data_mode)
{
	PT6315_DataSetCmd_t data_set_cmd;
	
	data_set_cmd.CMD = PT6315_DATA_CMD;
	data_set_cmd.SETTINGS_MODE = mode;
	data_set_cmd.ADDR_MODE = addr_mode;
	data_set_cmd.DATA_RW_MODE = data_mode;
	
	device->latch_control_fptr(0);
	device->spi_tx_data_fptr(&data_set_cmd.DataSettingCmd, 1);
	
	device->latch_control_fptr(1);
	//device->delay_fptr(1);
}

/*
 * @brief Sending address setting command.
 * 
 * @param device : instance of general data struct. 
 * @param address : set the addres of the display memory.
 *
 **/
static void PT6315_SetAddress(PT6315_GInst_t *device, uint8_t address)
{
	PT6315_AddressSetCmd_t addr_set_cmd;
	
	addr_set_cmd.CMD = PT6315_ADDR_CMD;
	addr_set_cmd.ADDR = address;
	
	device->latch_control_fptr(0);
	device->spi_tx_data_fptr(&addr_set_cmd.AddressCmd, 1);
	device->delay_fptr(1);
	device->latch_control_fptr(1);
}

