#ifndef _AUDIO_H_
#define _AUDIO_H_

/*
 * LINEAR TIME CODE (LTC)
 * https://en.wikipedia.org/wiki/Linear_timecode
 * Made up of 80 bits per frame, where there may be 24, 25 or 30 frames per second, 
 * LTC timecode varies from 960 Hz (binary zeros at 24 frames/s) to 2400 Hz (binary 
 * ones at 30 frames/s), and thus is comfortably in the audio frequency range. 
 * LTC can exist as either a balanced or unbalanced signal, and can be treated as an 
 * audio signal in regards to distribution. Like audio, LTC can be distributed by 
 * standard audio wiring, connectors, distribution amplifiers, and patchbays, 
 * and can be ground-isolated with audio transformers. It can also be distributed 
 * via 75 ohm video cable and video distribution amplifiers, although the voltage 
 * attenuation caused by using a 75 ohm system may cause the signal to drop to a 
 * level that can not be read by some equipment.
 * 
 * Vertical Interval Time Code (VITC) - burnt into analog video top lines
 * https://en.wikipedia.org/wiki/Vertical_interval_timecode
 * 
 * MIDI (Instrument) Time Code
 * https://en.wikipedia.org/wiki/MIDI_timecode
 * 
 * GenLock
 * https://en.wikipedia.org/wiki/Genlock
 */

/* MIC AND LINE LEVELS
 * 
 * https://soma.sbcc.edu/users/davega/FILMPRO_181_AUDIO_I/FILMPRO_181_04_Reference_Notes/FILMPRO_181_LineVSMicLevel/LINE%20LEVEL%20VS%20MIC%20LEVEL.pdf
 * Mic Level = (2 millivolts)
 * Line Level (pro) = +4db (1.23 volts)
 * Line Level (consumer) = -10db. (.316 volts)
 * 
 * https://www.kfs.oeaw.ac.at/manual/3.8/html/userguide/461.htm#:~:text=A%20line%20input%20level%20electrical,10%E2%80%935N%2Fm2).
 * A line input level electrical signal typically has a voltage 
 * ranging from 0,3 to 2 Volts, while a microphone level signal 
 * is more often in the range from 5 to 50 mV (millivolts). 
 * Microphone sensitivities range from -60 dBu to -22 dBu referenced 
 * to 94 dB Soud Pressure Level (0 dB SPL = 2*10–5N/m2).
 * The consumer line input level electrical signal typically has a 
 * voltage of 0,32 V (-7,8 dBu), whilst the professional line input 
 * level is typically 1,23 V (+4 dBu).
 * 
*/

/* More MIC AND LINE LEVELS info
 * https://www.sweetwater.com/insync/understanding-signal-levels-audio-gear/
 */

// http://www.sengpielaudio.com/calculator-db-volt.htm

// These are in RMS not the Peak-Peak. RMS is Vrms = 0.776 x Vp-p   (0.776 = sqrt(0.6))
// Line and Mic Vrms(mV)
#define mV_MIC_LOW_RMS 2.5          // -52dBv needs 42dB (125x) Gain to bring up to Comsumer Line Level
#define mV_MIC_HIGH_RMS 23.0        // -32dBv needs 22dB (12x) Gain to bring up to Comsumer Line Level
#define mV_LINE_CONSUMER_RMS 316.0  // -10dBv Needs 20dB (10x) Gain to get to 3.3v
#define mV_LINE_PRO_RMS 1228        // +4dBu (+1.78dBv)

// Line and Mic Vpeak-peak(mV)
#define mV_MIC_LOW_VPP 3.22         // x1024 gain to get to 3.3Vpp
#define mV_MIC_HIGH_VPP 29.6        // x143  gain to get to 3.3Vpp
#define mV_LINE_CONSUMER_VPP 407    // x8.1  gain to get to 3.3Vpp
#define mV_LINE_PRO_VPP 1582            // x2.08 gain to get to 3.3Vpp
#endif
