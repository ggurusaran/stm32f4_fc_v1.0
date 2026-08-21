/*
 * scheduler.c
 *
 *  Created on: Aug 21, 2026
 *      Author: gurus
 */

#include <flight_controller.h>
#include <scheduler.h>
#include <events.h>

extern FlightController_t fc;

void Scheduler_Tick(void)
{
	static uint32_t rc_counter;
	static uint32_t outer_pid_counter;

	Event_Set(EVENT_FLIGHT_CONTROL);
	Event_Set(EVENT_INNER_PID);

	rc_counter++;
	if(rc_counter>=10)
	{
		rc_counter=0;
		Event_Set(EVENT_RC_UPDATE);
	}
	outer_pid_counter++;
	if(outer_pid_counter>=4)
	{
		outer_pid_counter=0;
		Event_Set(EVENT_OUTER_PID);
	}

}

void Scheduler_Run(void)
{
	uint32_t events;

	events = Event_Get();
	/*
	 * Flight control
	 */

	if (events & EVENT_FLIGHT_CONTROL)
	{
		Event_Clear(EVENT_FLIGHT_CONTROL);

	}

	if (events & EVENT_INNER_PID)
	{
		Event_Clear(EVENT_INNER_PID);
		pid_inner_loop(&fc);
	}


	if (events & EVENT_OUTER_PID)
	{
		Event_Clear(EVENT_OUTER_PID);
		pid_outer_loop(&fc);

	}

	/**RC*/
	if (events & EVENT_RC_UPDATE)
	{
		Event_Clear(EVENT_RC_UPDATE);
		RC_Process(&fc.rawPWM, &fc.rc_cmd);

	}

}
