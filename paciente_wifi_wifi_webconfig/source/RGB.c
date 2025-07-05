/*
 * RGB.c
 * Definicion de funciones para manejo de encendido y apagado de RGB.
 *
 *  Created on: 13 mar. 2024
 *  Author: Daniel Alfaro y Frida Hernandez.
 */

#include "fsl_gpio.h"
#include "RW612.h"
#include "RGB.h"
#include "GPIO.h"
#include "bits.h"

//Funciones control led RGB
void RGB_on_red ()
{
	GPIO_PinWrite(GPIO, 0U, LED_RED_PIN, 0);
}

void RGB_on_blue ()
{
	GPIO_PinWrite(GPIO, 0U, LED_BLUE_PIN, 0);
}

void RGB_on_green ()
{
	GPIO_PinWrite(GPIO, 0U, LED_GREEN_PIN, 0);
}

void RGB_off_all ()
{
	GPIO_PinWrite(GPIO, 0U, LED_RED_PIN, 1);
	GPIO_PinWrite(GPIO, 0U, LED_GREEN_PIN, 1);
	GPIO_PinWrite(GPIO, 0U, LED_BLUE_PIN, 1);
}


void RGB_select_color_on(led_color_t color)
{
	switch(color)
	{
		case Red:
			RGB_off_all();
			RGB_on_red();
			break;
		case Blue:
			RGB_off_all();
			RGB_on_blue();
			break;
		case Green:
			RGB_off_all();
			RGB_on_green();
			break;
		case Purple:
			RGB_off_all();
			RGB_on_red();
			RGB_on_blue();
			break;
		case Yellow:
			RGB_off_all();
			RGB_on_red();
			RGB_on_green();
			break;
		case Cyan:
			RGB_off_all();
			RGB_on_blue();
			RGB_on_green();
			break;
		case White:
			RGB_on_red();
			RGB_on_blue();
			RGB_on_green();
			break;
		case Black:
			RGB_off_all();
			break;
		default:
			RGB_off_all();
	}
}

