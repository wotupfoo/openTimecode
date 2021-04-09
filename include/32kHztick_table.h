#pragma once

#ifndef _32kHztick_table_h_
#define _32kHztick_table_h_

#ifdef DEBUG
#include "debug_scope.h"
#endif

// Frame rates
// 23.98, 24, 25, 29.97, 29.97DF, 30

// Crude but it turns 24,25,30 into 0,1,2 offset in the trigger tick table.
const uint8_t fps_to_trigger_ticks_table [31] = {/*0..9*/  0,0,0,0,0,0,0,0,0,0,
										  /*10..19*/0,0,0,0,0,0,0,0,0,0,
										  /*20..23*/0,0,0,0,
										  /*24fps*/ 0,
										  /*25fps*/ 1,
										  /*26..29*/0,0,0,0,	
										  /*30fps*/ 2}; 

inline uint8_t fps_to_trigger_ticks(double fps) {
	if(fps == 23.98)	return 0;
	if(fps == 24.0)		return 1;
	if(fps == 25.0)		return 2;
	if(fps == 29.97)	return 3;
	if(fps == 30.0)		return 4;
#ifdef DEBUG
	DEBUGFAIL(LED_BUILTIN);
#endif
	return 4;	// Default to 30fps
}

const uint16_t trigger_ticks_32kHz[6][30] = {/* 32kHz tick value for a given frame number for each framerate in the range 0..29 framenumber */
		/*23.98  */{},
		/*24	 */{0,1365,2731,4096,5461,8192,9557,10923,12288,13653,15019,16384,17749,19115,20480,21845,23211,24576,25941,27307,28672,30037,31403,32768,34133,35499,36864,38229,39595},
		/*25     */{0,1311,2621,3932,5243,7864,9175,10486,11796,13107,14418,15729,17039,18350,19661,20972,22282,23593,24904,26214,27525,28836,30147,31457,32768,34079,35389,36700,38011},
		/*29.97  */{},
		/*30fps  */{0,1092,2185,3277,4369,6554,7646,8738,9830,10923,12015,13107,14199,15292,16384,17476,18569,19661,20753,21845,22938,24030,25122,26214,27307,28399,29491,30583,31676}
		};
#endif // _32kHztick_table_h_