/*
 * 	@file		bits.h
 *
 *  @Author 	L. Enrique Gortárez Ramírez, luis.gortarez@iteso.mx
 *
 *	brief
 *				Bit file for containing boolean types and 32 bit type
 *
 */

#ifndef BITS_H_
#define BITS_H_

/* Definitions for bits from 0 to 31 */
typedef enum
{
			  bit_0,  /* Bit 0 */
			  bit_1,  /* Bit 1 */
			  bit_2,  /* Bit 2 */
			  bit_3,  /* Bit 3 */
			  bit_4,  /* Bit 4 */
			  bit_5,  /* Bit 5 */
			  bit_6,  /* Bit 6 */
			  bit_7,  /* Bit 7 */
	          bit_8,  /* Bit 8 */
	          bit_9,  /* Bit 9 */
			  bit_10, /* Bit 10 */
			  bit_11, /* Bit 11 */
			  bit_12, /* Bit 12 */
			  bit_13, /* Bit 13 */
			  bit_14, /* Bit 14 */
			  bit_15, /* Bit 15 */
			  bit_16, /* Bit 16 */
			  bit_17, /* Bit 17 */
			  bit_18, /* Bit 18 */
			  bit_19, /* Bit 19 */
			  bit_20, /* Bit 20 */
			  bit_21, /* Bit 21 */
			  bit_22, /* Bit 22 */
			  bit_23, /* Bit 23 */
			  bit_24, /* Bit 24 */
			  bit_25, /* Bit 25 */
			  bit_26, /* Bit 26 */
			  bit_27, /* Bit 27 */
			  bit_28, /* Bit 28 */
			  bit_29, /* Bit 29 */
	          bit_30, /* Bit 30 */
			  bit_31  /* Bit 31 */
}	bits_t;

/* Boolean definitions */
typedef enum
{
			  FALSE,  /* 0 */
			  TRUE	  /* 1 */
} 	boolean_t;

#endif /* BITS_H_ */
