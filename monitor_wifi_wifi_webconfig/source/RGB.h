/*
 * Led.h
 * DEclaracion de funciones que permiten manipular el RGB de la K64.
 *
 *  Created on: 13 mar. 2024
 *  Author: Daniel Alfaro y Frida Hernandez.
 */

#ifndef RGB_H_
#define RGB_H_

#include <stdint.h>

#define LED_RED_PIN    1U
#define LED_GREEN_PIN  12U
#define LED_BLUE_PIN   0U

typedef enum
{
	Red,
	Blue,
	Green,
	Purple,
	Yellow,
	Cyan,
	White,
	Black
}led_color_t;

void RGB_on_red();
void RGB_on_blue();
void RGB_on_green();
void RGB_ToggleLed();

void RGB_off_all();
void RGB_select_color_on(led_color_t color);


//void RGB_red_on_off(uint32_t led);
//void RGB_blue_on_off(uint32_t led);
//void RGB_green_on_off(uint32_t led);

#endif /* RGB_H_ */
