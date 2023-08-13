/*
 * @brief Keyboard codes to ASCII source.
 * Created 07.29.23 by asw3005.
 */
#include "keycode.h"
#include "stm32f4xx.h"

/* Private variables. */

/*
 * @brief ASCII to key code decoding table.
 * Modification key + key code.
 *
 */
static const uint8_t ASCII_TO_KEYCODE[96][2] = {

	//   SPACE                    !                        "                                          #
	{KEY_MOD_NO, KEY_SPACE}, {KEY_MOD_LSHIFT, KEY_1}, {KEY_MOD_LSHIFT, KEY_APOSTROPHE_QUOTMARK}, {KEY_MOD_LSHIFT, KEY_3}, 									// 36
	//   $                        %                        &                        '
	{KEY_MOD_LSHIFT, KEY_4}, {KEY_MOD_LSHIFT, KEY_5}, {KEY_MOD_LSHIFT, KEY_7}, {KEY_MOD_NO, KEY_APOSTROPHE_QUOTMARK},										// 40
	//   (                        )                        *                        +
	{KEY_MOD_LSHIFT, KEY_9}, {KEY_MOD_LSHIFT, KEY_0}, {KEY_MOD_LSHIFT, KEY_8}, {KEY_MOD_LSHIFT, KEY_EQUAL_PLUS},
	//   ,                                 -                                   .                               /
	{KEY_MOD_NO, KEY_COMMA_LCHEVRON}, {KEY_MOD_NO, KEY_MINUS_UNDERSCORE}, {KEY_MOD_NO, KEY_DOT_RCHEVRON}, {KEY_MOD_NO, KEY_SLASH_QUESTMARK},				// 48
	//   0                    1                    2                    3
	{KEY_MOD_NO, KEY_0}, {KEY_MOD_NO, KEY_1}, {KEY_MOD_NO, KEY_2}, {KEY_MOD_NO, KEY_3},
	//   4                    5                    6                    7
	{KEY_MOD_NO, KEY_4}, {KEY_MOD_NO, KEY_5}, {KEY_MOD_NO, KEY_6}, {KEY_MOD_NO, KEY_7},																		// 56
	//   8                    9                      :                                      ;
	{KEY_MOD_NO, KEY_8}, {KEY_MOD_NO, KEY_9}, {KEY_MOD_LSHIFT, KEY_SEMICOLON_COLON}, {KEY_MOD_NO, KEY_SEMICOLON_COLON},
	//   <                                     =                             >                                   ?
	{KEY_MOD_LSHIFT, KEY_COMMA_LCHEVRON}, {KEY_MOD_NO, KEY_EQUAL_PLUS},	{KEY_MOD_LSHIFT, KEY_DOT_RCHEVRON}, {KEY_MOD_LSHIFT, KEY_SLASH_QUESTMARK},			// 64
	//   @                        A                        B                        C
	{KEY_MOD_LSHIFT, KEY_2}, {KEY_MOD_LSHIFT, KEY_a}, {KEY_MOD_LSHIFT, KEY_b}, {KEY_MOD_LSHIFT, KEY_c},
	//   D                        E                        F                        G
	{KEY_MOD_LSHIFT, KEY_d}, {KEY_MOD_LSHIFT, KEY_e}, {KEY_MOD_LSHIFT, KEY_f}, {KEY_MOD_LSHIFT, KEY_g},														// 72
	//	 H                        I                        J                        K
	{KEY_MOD_LSHIFT, KEY_h}, {KEY_MOD_LSHIFT, KEY_i}, {KEY_MOD_LSHIFT, KEY_j}, {KEY_MOD_LSHIFT, KEY_k},
	//   L                        M                        N                        O
	{KEY_MOD_LSHIFT, KEY_l}, {KEY_MOD_LSHIFT, KEY_m}, {KEY_MOD_LSHIFT, KEY_n}, {KEY_MOD_LSHIFT, KEY_o},														// 80
	//   P                        Q                        R                        S
	{KEY_MOD_LSHIFT, KEY_p}, {KEY_MOD_LSHIFT, KEY_q}, {KEY_MOD_LSHIFT, KEY_r}, {KEY_MOD_LSHIFT, KEY_s},
	//   T                        U                        V                        W
	{KEY_MOD_LSHIFT, KEY_t}, {KEY_MOD_LSHIFT, KEY_u}, {KEY_MOD_LSHIFT, KEY_v}, {KEY_MOD_LSHIFT, KEY_w},														// 88
	//   X                        Y                        Z                        [
	{KEY_MOD_LSHIFT, KEY_x}, {KEY_MOD_LSHIFT, KEY_y}, {KEY_MOD_LSHIFT, KEY_z}, {KEY_MOD_NO, KEY_LBRACKET_LBRACE},
	//   \                                 ]                                   ^                        _
	{KEY_MOD_NO, KEY_BACKSLASH_VBAR}, {KEY_MOD_NO, KEY_RBRACKET_RBRACE}, {KEY_MOD_LSHIFT, KEY_6}, {KEY_MOD_LSHIFT, KEY_MINUS_UNDERSCORE},					// 96
	//   `
	{KEY_MOD_NO, KEY_GRAVE_TILDE},
	//   a                    b                    c
	{KEY_MOD_NO, KEY_a}, {KEY_MOD_NO, KEY_b}, {KEY_MOD_NO, KEY_c},
	//   d                    e                    f                    g
	{KEY_MOD_NO, KEY_d}, {KEY_MOD_NO, KEY_e}, {KEY_MOD_NO, KEY_f}, {KEY_MOD_NO, KEY_g},																		// 104
	//   h                    i                    j                    k
	{KEY_MOD_NO, KEY_h}, {KEY_MOD_NO, KEY_i}, {KEY_MOD_NO, KEY_j}, {KEY_MOD_NO, KEY_k},
	//   l                    m                    n                    o
	{KEY_MOD_NO, KEY_l}, {KEY_MOD_NO, KEY_m}, {KEY_MOD_NO, KEY_n}, {KEY_MOD_NO, KEY_o},																		// 112
	//   p                    q                    r                    s
	{KEY_MOD_NO, KEY_p}, {KEY_MOD_NO, KEY_q}, {KEY_MOD_NO, KEY_r}, {KEY_MOD_NO, KEY_s},
	//   t                    u                    v                    w
	{KEY_MOD_NO, KEY_t}, {KEY_MOD_NO, KEY_u}, {KEY_MOD_NO, KEY_v}, {KEY_MOD_NO, KEY_w},																		// 120
	//   x                    y                    z                    {
	{KEY_MOD_NO, KEY_x}, {KEY_MOD_NO, KEY_y}, {KEY_MOD_NO, KEY_z}, {KEY_MOD_LSHIFT, KEY_LBRACKET_LBRACE},
	//   |                                     }                                      ~
	{KEY_MOD_LSHIFT, KEY_BACKSLASH_VBAR}, {KEY_MOD_LSHIFT, KEY_RBRACKET_RBRACE}, {KEY_MOD_LSHIFT, KEY_GRAVE_TILDE}, {KEY_MOD_NO, KEY_NONE}					// 128

};

/*
 * @brief Getting keyboard code from the key ASCII code.
 */
OutCode_t* KEY_GetCode(char InputChar) {


	static OutCode_t OutCode = { 0 };

	if ((InputChar - ASCII_CHAR_OFFSET) > 96) {
		OutCode.KeyMode = KEY_MOD_NO;
		OutCode.KeyCode = 0x04;
		return &OutCode;
	}

	OutCode.KeyMode = ASCII_TO_KEYCODE[(uint8_t)InputChar - ASCII_CHAR_OFFSET][0];
	OutCode.KeyCode = ASCII_TO_KEYCODE[(uint8_t)InputChar - ASCII_CHAR_OFFSET][1];

	return &OutCode;
}

/*
 * @brief Decoding password number to its string representation.
 */
void PASS_NumberToString(char* InBuff) {




}








































