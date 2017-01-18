/* Copyright (c) 2015 Graham Kelly
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
 
#ifndef BNE_EMG_V1_5_H
#define BNE_EMG_V1_5_H

// Board has no LEDs or buttons
#define LEDS_NUMBER    0
#define BUTTONS_NUMBER 0
#define HWFC           false


//These are the correct pin definitions
/**/
#define SPI_INSTANCE 				0
#define SPI0_ENABLED				1
#define SPI0_USE_EASY_DMA		0
#define SPIM0_SCK_PIN      	13    
#define SPIM0_MOSI_PIN      14    
#define SPIM0_MISO_PIN      12    
#define SPIM0_SS_PIN        15

#define MPU_TWI_SCL_PIN 5
#define MPU_TWI_SDA_PIN 6

#ifdef S210
#define NRF_CLOCK_LFCLKSRC      NRF_CLOCK_LFCLKSRC_XTAL_20_PPM
#else
#define NRF_CLOCK_LFCLKSRC  {	.source        = NRF_CLOCK_LF_SRC_RC,            	\
															.rc_ctiv = 2, 																		\
															.rc_temp_ctiv = 0, 																\
															.xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_250_PPM}
#endif


#endif // BNE_EMG_V1_5_H
