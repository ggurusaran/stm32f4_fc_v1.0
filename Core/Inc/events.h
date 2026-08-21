/*
 * events.h
 *
 *  Created on: Aug 21, 2026
 *      Author: gurus
 */

#ifndef INC_EVENTS_H_
#define INC_EVENTS_H_


#include <stdint.h>

/* Periodic events */
#define EVENT_RC_UPDATE        (1U << 0)
#define EVENT_FLIGHT_CONTROL   (1U << 1)
#define EVENT_OUTER_PID        (1U << 2)
#define EVENT_INNER_PID        (1U << 3)

/* Future asynchronous events */
#define EVENT_IMU_READY        (1U << 4)
#define EVENT_UART_RX          (1U << 5)
#define EVENT_FAILSAFE         (1U << 6)


void Event_Set(uint32_t event);
void Event_Clear(uint32_t event);
uint32_t Event_Get(void);

#endif /* INC_EVENTS_H_ */
