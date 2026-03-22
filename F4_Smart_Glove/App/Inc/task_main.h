/*
 * task_main.h
 *
 *  Created on: Mar 12, 2026
 *      Author: lenovo
 */

#ifndef INC_TASK_MAIN_H_
#define INC_TASK_MAIN_H_

#include "main.h"
#include "usart.h"
#include "i2c.h"
#include "time.h"
//#include "tca9548a.h"
//#include "gesture.h"

void MainTask_Init();
void MainTask_Loop();

#endif /* INC_TASK_MAIN_H_ */
