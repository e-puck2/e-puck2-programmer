/**
 * @file	uc_usage.c
 * @brief  	Functions to show the uC usage
 * 
 * @source			http://www.chibios.com/forum/viewtopic.php?f=2&t=138&start=10
 * @modified by  	Eliot Ferragni
 */
 
#include "main.h"

//////////////////////////////////////////PUBLIC FUNCTIONS/////////////////////////////////////////

void printUcUsage(BaseSequentialStream* out) {
	
	thread_t *tp;

	static uint64_t sum;
	static uint16_t tmp1, tmp2;
	sum = 0;
	//takes the first thread created
	tp = chRegFirstThread();
	//computes the total number of cycles counted
	do {
		sum += tp->stats.cumulative;
		tp = chRegNextThread(tp);
	} while (tp != NULL);
	sum += ch.kernel_stats.m_crit_thd.cumulative;
	sum += ch.kernel_stats.m_crit_isr.cumulative;

	//takes the first thread created
	tp = chRegFirstThread();
	//computes the percentage of time used by eahc thread
	do {
		tmp1 = (uint16_t)(tp->stats.cumulative*10000/sum);
		chprintf(out, "%30s %u.%u%%\r\n", tp->name, tmp1/100, tmp1%100);
		tp = chRegNextThread(tp);
	} while (tp != NULL);

	tmp1 = (uint16_t)(ch.kernel_stats.m_crit_thd.cumulative*10000/sum);
	tmp2 = (uint16_t)(ch.kernel_stats.m_crit_isr.cumulative*10000/sum);

	chprintf(out, "critical thd:%u.%u%%   critical isr:%u.%u%%\r\n",
	  tmp1/100, tmp1%100,tmp2/100, tmp2%100);
	chprintf(out, "\r\n");
}