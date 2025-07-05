/*
 * GPIO.c
 * Se definen las funciones con las cuales se configuran y controlan los pines
 * de la tarjeta.
 *
 *  Created on: 26 ene. 2024
 *  Author: Daniel Alfaro y Frida Hernandez.
 */

#include "fsl_common.h"
#include "fsl_gpio.h"
#include "fsl_io_mux.h"
#include "pin_mux.h"
#include "stdbool.h"

#include "Bits.h"
#include "GPIO.h"
#include "RGB.h"


static uint8_t gpio_swflag = 0;

//Funciones de configuracion de puerto y pin
void GPIO_init(void)
{
	//Activar clock de cada puerto
	CLOCK_EnableClock(kCLOCK_HsGpio0);
	CLOCK_EnableClock(kCLOCK_HsGpio1);

	GPIO_PortInit(GPIO, 0U);
	GPIO_PortInit(GPIO, 1U);  // si tus pines están en el port 1

	IO_MUX_SetPinMux(IO_MUX_GPIO0);
	IO_MUX_SetPinMux(IO_MUX_GPIO1);
	IO_MUX_SetPinMux(IO_MUX_GPIO12);

	IO_MUX_SetPinMux(IO_MUX_GPIO25);

	gpio_pin_config_t led_config =
	{
		kGPIO_DigitalOutput, 1U
	};

	// Botón SW2 (GPIO0_11 como entrada)
	gpio_pin_config_t sw2_button_config =
	{
	    kGPIO_DigitalInput, 0U
	};

	// Inicializa los LEDs como salida
	GPIO_PinInit(GPIO, 0U, LED_RED_PIN, &led_config);
	GPIO_PinInit(GPIO, 0U, LED_GREEN_PIN, &led_config);
	GPIO_PinInit(GPIO, 0U, LED_BLUE_PIN, &led_config);

	GPIO_PinInit(GPIO, 0U, 11U, &sw2_button_config);

	/* Enables the clock for the GPIO0 module */
//	GPIO_PortInit(GPIO, 0);
//
//	gpio_pin_config_t gpio0_pinD12_config = {
//		.pinDirection = kGPIO_DigitalOutput,
//		.outputLogic = 0U
//	};
//	/* Initialize GPIO functionality on pin PIO0_0 (pin D12)  */
//	GPIO_PinInit(GPIO, 0U, 0U, &gpio0_pinD12_config);
//	/* Initialize FC3_USART_DATA functionality on pin GPIO_24 (pin F3) */
//	IO_MUX_SetPinMux(IO_MUX_FC3_USART_DATA);
//	/* Initialize GPIO0 functionality on pin GPIO_0 (pin D12) */
//	IO_MUX_SetPinMux(IO_MUX_GPIO0);
//	/* Initialize GPIO25 functionality on pin GPIO_25 (pin G3) */
//	IO_MUX_SetPinMux(IO_MUX_GPIO25);
}

void GPIO_SetFlag()
{
	gpio_swflag = 1;
}

void GPIO_ClearFlag()
{
	gpio_swflag = 0;
}

uint8_t GPIO_GetFlag()
{
	return gpio_swflag;
}
