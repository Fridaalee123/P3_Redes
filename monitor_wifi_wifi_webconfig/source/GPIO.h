/*
 * GPIO.h
 * Declaración de funciones, estructuras y direcciones para manipular y confugurar
 * los puertos de la k64.
 *
 *  Created on: 26 ene. 2024
 *   Author: Daniel Alfaro y Frida Hernandez.
 */

#ifndef GPIO_H_
#define GPIO_H_

#include "fsl_gpio.h"
#include "stdint.h"
#include "bits.h"

#define value_zero 	0U
#define turn_on		1U
#define clean_all 	0xFFFFFFFF

#define SW2_PORT 0U
#define SW2_PIN  11U

//Tipo enumerado con valores del 0 al 9
typedef enum
{
	Zero,
	One,
	Two,
	Three,
	Four,
	Five,
	Six,
	Seven,
	Eight,
	Nine
}number_name_t;

void GPIO_init(void);
void GPIO_SetFlag();
void GPIO_ClearFlag();
uint8_t GPIO_GetFlag();


#endif /* GPIO_H_ */
