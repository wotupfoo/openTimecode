#pragma once

#ifndef _DEBUG_SCOPE_H_
#define _DEBUG_SCOPE_H_

#define BUFF_DEBUG_SIZE 4096
static char buff_debug[BUFF_DEBUG_SIZE];

void inline SCOPESET(uint8_t x) {
#ifdef DEBUG
	pinMode(x,OUTPUT);
	digitalWrite(x,LOW);
#endif
}

void inline SCOPE(uint8_t x) {
#ifdef DEBUG
	digitalWrite(x,HIGH);
	digitalWrite(x,LOW);
#endif
}

void inline SCOPEON(uint8_t x) {
#ifdef DEBUG
	digitalWrite(x,HIGH);
#endif
}

void inline SCOPEOFF(uint8_t x) {
#ifdef DEBUG
	digitalWrite(x,LOW);
#endif
}

void DEBUGFAIL(uint8_t x) {
#ifdef DEBUG
	for(;;) { // Don't proceed, loop forever
        SCOPEON(x);
		delay(100);
        SCOPEOFF(x);
      }
#endif
}

#if 0
void vTaskGetRunTimeStats( char* pcWriteBuffer )
{
TaskStatus_t *pxTaskStatusArray;
volatile UBaseType_t uxArraySize, x;
uint32_t ulTotalRunTime, ulStatsAsPercentage;

	 /* Make sure the write buffer does not contain a string. */
	 *pcWriteBuffer = 0x00;

	 /* Take a snapshot of the number of tasks in case it changes while this
	 function is executing. */
	 uxArraySize = uxTaskGetNumberOfTasks();

	 /* Allocate a TaskStatus_t structure for each task.  An array could be
	 allocated statically at compile time. */
	 pxTaskStatusArray = (TaskStatus_t*)pvPortMalloc( uxArraySize * sizeof( TaskStatus_t ) );

	 if( pxTaskStatusArray != NULL )
	 {
			/* Generate raw status information about each task. */
			uxArraySize = uxTaskGetSystemState( pxTaskStatusArray,
																 uxArraySize,
																 &ulTotalRunTime );

			/* For percentage calculations. */
			ulTotalRunTime /= 100UL;

			/* Avoid divide by zero errors. */
			if( ulTotalRunTime > 0 )
			{
				 /* For each populated position in the pxTaskStatusArray array,
				 format the raw data as human readable ASCII data. */
				 for( x = 0; x < uxArraySize; x++ )
				 {
						/* What percentage of the total run time has the task used?
						This will always be rounded down to the nearest integer.
						ulTotalRunTimeDiv100 has already been divided by 100. */
						ulStatsAsPercentage =
									pxTaskStatusArray[ x ].ulRunTimeCounter / ulTotalRunTime;

						if( ulStatsAsPercentage > 0UL )
						{
							 sprintf( pcWriteBuffer, "%s \t%8d %d%%\r\n",
																 pxTaskStatusArray[ x ].pcTaskName,
																 pxTaskStatusArray[ x ].ulRunTimeCounter,
																 ulStatsAsPercentage );
						}
						else
						{
							 /* If the percentage is zero here then the task has
							 consumed less than 1% of the total run time. */
							 sprintf( pcWriteBuffer, "%s \t%8d <1%%\r\n",
																 pxTaskStatusArray[ x ].pcTaskName,
																 pxTaskStatusArray[ x ].ulRunTimeCounter );
						}

						pcWriteBuffer += strlen( ( char * ) pcWriteBuffer );
				 }
			}

			/* The array is no longer needed, free the memory it consumes. */
			vPortFree( pxTaskStatusArray );
	 }
}
#endif

#endif // _DEBUG_SCOPE_H_
