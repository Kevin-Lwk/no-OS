/***************************************************************************//**
 *   @file   mqtt_noos_support.c
 *   @brief  Implementation file used to port the MQTT paho to use no-os
 *   @author Mihail Chindris (mihail.chindris@analog.com)
********************************************************************************
 * Copyright 2020(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include "mqtt_noos_support.h"
#include <stdlib.h>
#include "no-os/timer.h"
#include "no-os/error.h"
#include "no-os/util.h"
#include "no-os/delay.h"
#include "no-os/error.h"

/******************************************************************************/
/**************************** Global Variables ********************************/
/******************************************************************************/

/* Timer reference used by the functions */
static struct timer_desc	*timer;

/* Number of timer references */
static uint32_t			nb_references;

/******************************************************************************/
/************************ Functions Definitions *******************************/
/******************************************************************************/

/* Allcoate resources for timer. Must be called from mqqt_init */
int32_t mqtt_timer_init(uint32_t timer_id, void *extra_init_param)
{
	struct timer_init_param init_param = {
		.id = timer_id,
		.freq_hz = 1000,
		.load_value = 0,
		.extra = extra_init_param
	};
	int32_t			ret;

	ret = timer_init(&timer, &init_param);
	if (IS_ERR_VALUE(ret)) {
		timer = NULL;
		return FAILURE;
	}

	ret = timer_start(timer);
	if (IS_ERR_VALUE(ret)) {
		timer_remove(timer);
		timer = NULL;
		return FAILURE;
	}

	nb_references++;

	return SUCCESS;
}


/* Remove resources allocated with \ref mqtt_timere_init. */
void mqtt_timer_remove()
{
	nb_references--;
	if (!nb_references) {
		timer_remove(timer);
		timer = NULL;
	}
}

/* Implementation of TimerInit used by MQTTClient.c */
void TimerInit(Timer* t)
{
	UNUSED_PARAM(t);
	/* Do nothing */
}

/* Implementation of TimerCountdownMS used by MQTTClient.c */
void TimerCountdownMS(Timer* t, unsigned int ms)
{
	timer_counter_get(timer, &t->start_time);
	t->ms = ms;
}

/* Implementation of TimerCountdown used by MQTTClient.c */
void TimerCountdown(Timer* t, unsigned int seconds)
{
	timer_counter_get(timer, &t->start_time);
	t->ms = seconds * 1000;
}

/* Implementation of TimerLeftMS used by MQTTClient.c */
int TimerLeftMS(Timer* t)
{
	uint32_t ms;

	timer_counter_get(timer, &ms);
	ms -= t->start_time;

	if (ms > t->ms)
		return 0;
	else
		return (t->ms - ms);
}

/* Implementation of TimerIsExpired used by MQTTClient.c */
char TimerIsExpired(Timer* t)
{
	if (TimerLeftMS(t) == 0)
		return true;

	return false;
}

/* Implementation of mqtt_noos_read used by MQTTClient.c */
int mqtt_noos_read(Network* net, unsigned char* buff, int len, int timeout)
{
	uint32_t	sent;
	int32_t		rc;

	if (!len)
		return 0;

	sent = 0;
	do {
		rc = socket_recv(net->sock, (void *)(buff + sent),
				 (uint32_t)(len - sent));
		if (rc != -EAGAIN) { //If data available or error
			if (IS_ERR_VALUE(rc))
				return rc;

			sent += rc;
			if (sent >= len)
				return sent;
		}

		mdelay(1);
	} while (--timeout);

	/* 0 bytes have been read */
	return 0;
}

/* Implementation of mqtt_noos_write used by MQTTClient.c */
int mqtt_noos_write(Network* net, unsigned char* buff, int len, int timeout)
{
	/* non blocking read is not implemented */
	UNUSED_PARAM(timeout);

	return socket_send(net->sock, (const void *)buff, (uint32_t)len);
}
