/*
 * @brief Keyboard codes to ASCII header.
 * Created 07.29.23 by asw3005.
 */
#include "stm32f4xx.h"

#ifndef KEYCODE_H_
#define KEYCODE_H_

//#define NO_LCD

#define	KEY_NONE 			0x00
#define	KEY_ERR_ROLL_OVER 	0x01
#define	KEY_POST_FAIL		0x02
#define	KEY_ERR_UNDEFINED	0x03

#define ASCII_CHAR_OFFSET  	0x20

/*
 * @brief Key modifier masks.
 */
typedef enum {
	KEY_MOD_NO 			= 0x00,
	KEY_MOD_LCTRL 		= 0x01,
	KEY_MOD_LSHIFT 		= 0x02,
	KEY_MOD_LALT 		= 0x04,
	KEY_MOD_LGUI 		= 0x08,
	KEY_MOD_RCTRL 		= 0x10,
	KEY_MOD_RSHIFT 		= 0x20,
	KEY_MOD_RALT 		= 0x40,
	KEY_MOD_RGUI 		= 0x80

} MODIFIER_KEYS_t;

/*
 * @brief Control keys.
 */
typedef enum {
	KEY_LCTRL 			= 0xE0,
	KEY_LSHIFT,
	KEY_LALT,
	KEY_LGUI,
	KEY_RCTRL,
	KEY_RSHIFT,
	KEY_RALT,
	KEY_RGUI

} CTRL_KEYS_t;

/*
 * @brief Number keys.
 */
typedef enum {
	/* Keys 1 and !. */
	KEY_1 				= 0x1E,
	/* Keys 2 and @. */
	KEY_2,
	/* Keys 3 and #. */
	KEY_3,
	/*  Keys 4 and $. */
	KEY_4,
	/* Keys 5 and %. */
	KEY_5,
	/* Keys 6 and ^. */
	KEY_6,
	/* Keys 7 and &. */
	KEY_7,
	/* Keys 8 and *. */
	KEY_8,
	/* Keys 9 and (. */
	KEY_9,
	/* Keys 0 and ). */
	KEY_0

} NUMBER_KEYS_t;


/*
 * @brief Character keys.
 */
typedef enum {
	KEY_a 				= 0x04,
	KEY_b,
	KEY_c,
	KEY_d,
	KEY_e,
	KEY_f,
	KEY_g,
	KEY_h,
	KEY_i,
	KEY_j,
	KEY_k,
	KEY_l,
	KEY_m,
	KEY_n,
	KEY_o,
	KEY_p,
	KEY_q,
	KEY_r,
	KEY_s,
	KEY_t,
	KEY_u,
	KEY_v,
	KEY_w,
	KEY_x,
	KEY_y,
	KEY_z

} LETTER_KEYS_t;

/*
 * @brief Functional keys.
 */
typedef enum {
	KEY_F1 				= 0x3a,
	KEY_F2,
	KEY_F3,
	KEY_F4,
	KEY_F5,
	KEY_F6,
	KEY_F7,
	KEY_F8,
	KEY_F9,
	KEY_F10,
	KEY_F11,
	KEY_F12

} FUNCTIONAL_KEYS_t;

/*
 * @brief Special characters and keys.
 */
typedef enum {
	KEY_ENTER 			= 0x28,
	KEY_ESC,
	KEY_BACKSPACE,
	KEY_TAB,
	KEY_SPACE,
	/* Key - and _ */
	KEY_MINUS_UNDERSCORE,
	/* Key = and + */
	KEY_EQUAL_PLUS,
	/* Key [ and {. */
	KEY_LBRACKET_LBRACE,
	/* Key ] and }. */
	KEY_RBRACKET_RBRACE,
	/* Key \ and |. */
	KEY_BACKSLASH_VBAR,
	/* Key # and ~ Non-US. */
	KEY_HASH_TILDE,
	/* Key ; and :. */
	KEY_SEMICOLON_COLON,
	/* Key ' and ". */
	KEY_APOSTROPHE_QUOTMARK,
	/* Key ` and ~. */
	KEY_GRAVE_TILDE,
	/* Key , and <. */
	KEY_COMMA_LCHEVRON,
	/* Key . and >. */
	KEY_DOT_RCHEVRON,
	/* Key / and ?. */
	KEY_SLASH_QUESTMARK,
	KEY_CAPSLOCK,
	KEY_PRINT_SCREEN 	= 0x46,
	KEY_SCROLLLOCK,
	KEY_PAUSE,
	KEY_INSERT,
	KEY_HOME,
	KEY_PAGEUP,
	KEY_DELETE,
	KEY_END,
	KEY_PAGEDOWN,
	KEY_RIGHT,
	KEY_LEFT,
	KEY_DOWN,
	KEY_UP,
	KEY_NUMLOCK,
	/* Key /. */
	KEY_KPSLASH,
	/* Key *. */
	KEY_KPASTERISK,
	/* Key -. */
	KEY_KPMINUS,
	/* Key +. */
	KEY_KPPLUS,
	KEY_KPENTER,
	KEY_KP1_END,
	KEY_KP2_DNARROW,
	KEY_KP3_PAGEDN,
	KEY_KP4_LARROW,
	KEY_KP5,
	KEY_KP6_RARROW,
	KEY_KP7_HOME,
	KEY_KP8_UPARROW,
	KEY_KP9_PAGEUP,
	KEY_KP0_INSERT,
	KEY_KPDOT_DELETE,
	KEY_MEDIA_PLAYPAUSE = 0xE8,
	KEY_MEDIA_STOPCD,
	KEY_MEDIA_PREVIOUSSONG,
	KEY_MEDIA_NEXTSONG,
	KEY_MEDIA_EJECTCD,
	KEY_MEDIA_VOLUMEUP,
	KEY_MEDIA_VOLUMEDOWN,
	KEY_MEDIA_MUTE,
	KEY_MEDIA_WWW,
	KEY_MEDIA_BACK,
	KEY_MEDIA_FORWARD,
	KEY_MEDIA_STOP,
	KEY_MEDIA_FIND,
	KEY_MEDIA_SCROLLUP,
	KEY_MEDIA_SCROLLDOWN,
	KEY_MEDIA_EDIT,
	KEY_MEDIA_SLEEP,
	KEY_MEDIA_COFFEE,
	KEY_MEDIA_REFRESH,
	KEY_MEDIA_CALC

} SPECIAL_KEYS_t;

/*
 * @brief Keys' code returning type.
 */
typedef struct {

	uint8_t KeyMode;
	uint8_t KeyCode;

} OutCode_t;



/* Public function prototypes. */
OutCode_t* KEY_GetCode(char InputChar);



#endif /* KEYCODE_H_ */
