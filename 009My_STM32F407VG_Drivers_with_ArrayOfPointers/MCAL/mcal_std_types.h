/*
 * mcal_std_types.h
 *
 *  Created on: Sep 18, 2023
 *      Author: mohamed
 */

#ifndef MCAL_STD_TYPES_H_
#define MCAL_STD_TYPES_H_

/* Section : Includes */
#include "std_libraries.h"

#define ENABLE          1           /**< Enable macro */
#define DISABLE         0           /**< Disable macro */
#define SET             ENABLE      /**< Set macro */
#define RESET           DISABLE     /**< Reset macro */
#define GPIO_PIN_SET    SET         /**< GPIO Pin set macro */
#define GPIO_PIN_RESET  RESET       /**< GPIO Pin reset macro */
#define FLAG_SET        SET         /**< Flag set macro */
#define FLAG_RESET      RESET       /**< Flag reset macro */

// Define common data types

// Unsigned integer data types
/*
typedef unsigned char               uint8;
typedef unsigned short              uint16;
typedef unsigned int                uint32;
typedef unsigned long long int      uint64;
*/

typedef uint8_t  uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;
typedef uint64_t uint64;

// Signed integer data types
typedef signed char                 sint8_t;
typedef signed short                sint16_t;
typedef signed int                  sint32_t;
typedef signed long long int        sint64_t;

// volatile unsigned integer data types
typedef volatile unsigned char              vuint8_t;
typedef volatile unsigned short             vuint16_t;
typedef volatile unsigned int               vuint32_t;
typedef volatile unsigned long long int     vuint64_t;

// volatile signed integer data types
typedef volatile signed char                vsint8_t;
typedef volatile signed short               vsint16_t;
typedef volatile signed int                 vsint32_t;
typedef volatile signed long long int       vsint64_t;

// Float data types
typedef float       float32;
typedef double      float64;

typedef uint8 Std_ReturnType;

/* Section : Macro Declarations */
#define E_OK            (Std_ReturnType)0x01
#define E_NOT_OK        (Std_ReturnType)0x00

#define ZERO_INIT 0


// Define common macros
#define BIT_MASK (uint8)1

#define GET_BIT(REG, BIT_POSN)      (((REG) >> (BIT_POSN)) & 1)
#define SET_BIT(REG,BIT_POSN)       ((REG) |= (BIT_MASK << BIT_POSN))
#define CLEAR_BIT(REG,BIT_POSN)     ((REG) &= ~(BIT_MASK << BIT_POSN))
#define TOGGLE_BIT(REG,BIT_POSN)    ((REG) ^= (BIT_MASK << BIT_POSN))
#define READ_BIT(REG, BIT_POSN)     (((REG) & (BIT_MASK << (BIT_POSN))) != 0)
#define MODIFY_REG(REG, MASK, VAL)  ((REG) = ((REG) & (~(MASK))) | (VAL))

#define GET_REG(REG)                (REG)
#define SET_REG(REG)                ((REG) = 0xFF)
#define CLR_REG(REG)                ((REG) = 0x00)
#define TOGGLE_REG(REG)             ((REG) ^= 0xFF)
#define ASSIGN_REG(REG, VALUE)      ((REG) = (VALUE))

#define GET_LOW_NIB(REG)            ((REG) & 0x0F)
#define SET_LOW_NIB(REG)            ((REG) |= 0x0F)
#define CLR_LOW_NIB(REG)            ((REG) &= 0xF0)
#define TOGGLE_LOW_NIB(REG)         ((REG) ^= 0x0F)
#define ASSIGN_LOW_NIB(REG, VALUE)  ((REG) = (((REG) & 0xF0) | ((VALUE) & 0x0F)))

#define GET_HIGH_NIB(REG)           (((REG) & 0xF0) >> 4)
#define SET_HIGH_NIB(REG)           ((REG) |= 0xF0)
#define CLR_HIGH_BIT(REG)           ((REG) &= 0x0F)
#define TOGGLE_HIGH_NIB(REG)        ((REG) ^= 0xF0)
#define ASSIGN_HIGH_NIB(REG, VALUE) ((REG) = (((VALUE) << 4) | ((REG) & 0x0F)))

#define NULL                        ((void *)0)

// Boolean date types
typedef enum
{
	False,
	True
}Bool;

// Error types which used in ErrorIndication Function in any Default Case in Switch
typedef enum
{
	InvalidArgument,    //in any configuration to Pins or Ports
	OverFlow            //in any configuration to LEDs For Example
}ErrorType;


#endif /* MCAL_STD_TYPES_H_ */
