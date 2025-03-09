/*
 * encoder.c
 *
 *  Created on: Jun 28, 2024
 *      Author: yhy
 */
#include "main.h"

int encoder_read(TIM_HandleTypeDef *htim)
{
	int temp;
	temp = (short)__HAL_TIM_GET_COUNTER(htim);
	__HAL_TIM_SET_COUNTER(htim,0);
	return temp;
}
