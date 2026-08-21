/*
 * scheduler.h
 *
 *  Created on: Aug 21, 2026
 *      Author: gurus
 */

#ifndef INC_SCHEDULER_H_
#define INC_SCHEDULER_H_


#include <stdint.h>

void Scheduler_Init(void);

/* Called every 1 ms from TIM6 ISR */
void Scheduler_Tick(void);

/* Called continuously from main loop */
void Scheduler_Run(void);

#endif /* INC_SCHEDULER_H_ */
