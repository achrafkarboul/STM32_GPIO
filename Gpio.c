/*----------------------------------------------------------------------------
 * Name:    Gpio.c
 * Purpose: GPIO usage for STM32
 * Version: V1.00
 *----------------------------------------------------------------------------
 * This file is part of the uVision/ARM development tools.
 * This software may only be used under the terms of a valid, current,
 * end user licence from KEIL for a compatible version of KEIL software
 * development tools. Nothing else gives you the right to use this software.
 *
 * Copyright (c) 2005-2007 Keil Software. All rights reserved.
 *----------------------------------------------------------------------------*/

#include <stm32f10x_lib.h>                        // STM32F10x Library Definitions
#include "STM32_Init.h"                           // STM32 Initialization

#define S3             0x2000                     // PC13: S3
#define S2             0x0001                     // PA0 : S2
#define UNBOUNCE_CNT        5                     // unbounce the keys

int ledPosCur = 3;                                // current led position from 0..7
int ledPosOld = 3;                                // old     led position from 0..7


/*----------------------------------------------------------------------------
  S2Pressed
  check if S2 is pressed (unbounced).
 *----------------------------------------------------------------------------*/
int S2Pressed (void) {
  static int S2KeyCount = 0, S2KeyPressed = 0;

  if (S2KeyPressed) {
    if (!((GPIOA->IDR & S2) == 0 )) {             // Check if S2 is not pressed
      if (S2KeyCount < UNBOUNCE_CNT) S2KeyCount++;
      else {
        S2KeyPressed = 0;
        S2KeyCount = 0;    
      }
    }
  }
  else {
    if (((GPIOA->IDR & S2) == 0 ))  {             // Check if S2 is pressed
      if (S2KeyCount < UNBOUNCE_CNT) S2KeyCount++;
      else {
        S2KeyPressed = 1;
        S2KeyCount = 0;
		return (1);
      }
    }
  }
  return (0);
}


/*----------------------------------------------------------------------------
  S3Pressed
  check if S3 is pressed (unbounced).
 *----------------------------------------------------------------------------*/
int S3Pressed (void) {
  static int S3KeyCount = 0, S3KeyPressed = 0;

  if (S3KeyPressed) {
    if (!((GPIOC->IDR & S3) == 0 )) {             // Check if S3 is not pressed
      if (S3KeyCount < UNBOUNCE_CNT) S3KeyCount++;
      else {
        S3KeyPressed = 0;
        S3KeyCount = 0;    
      }
    }
  }
  else {
    if (((GPIOC->IDR & S3) == 0 ))  {             // Check if S3 is pressed
      if (S3KeyCount < UNBOUNCE_CNT) S3KeyCount++;
      else {
        S3KeyPressed = 1;
        S3KeyCount = 0;
		return (1);
      }
    }
  }
  return (0);
}


/*----------------------------------------------------------------------------
  MAIN function
 *----------------------------------------------------------------------------*/
int main (void) {

  stm32_Init ();                                  // STM32 setup
    
  GPIOB->ODR |=  (1 << (ledPosCur+8));            // switch on initial LED position

  while (1) {                                     // Loop forever

    if (S2Pressed()) {                            //  S2 pressed
      if (ledPosCur > 0) 
	    ledPosCur -= 1;                           // led position moves to right
    }

    if (S3Pressed()) {                            //  S3 pressed
      if (ledPosCur < 7) 
	    ledPosCur += 1;                           // led position moves to left
    }

    if (ledPosCur != ledPosOld) {                 // LED osition has changed
      GPIOB->ODR &= ~(1 << (ledPosOld+8));        // switch off    old LED position
      GPIOB->ODR |=  (1 << (ledPosCur+8));        // switch on  current LED position
      ledPosOld = ledPosCur;
    }
  } // end while
} // end main
