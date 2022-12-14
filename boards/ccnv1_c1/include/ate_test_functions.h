/* ----------------------------------------------------------------------
*
* Copyright (c) 2017 ChipCraft Sp. z o.o. All rights reserved
*
* $Date: 2022-03-15 09:30:11 +0100 (wto, 15 mar 2022) $
* $Revision: 828 $
*
*  ----------------------------------------------------------------------
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*   - Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   - Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in
*     the documentation and/or other materials provided with the
*     distribution.
*   - Neither the name of ChipCraft Sp. z o.o. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
 * -------------------------------------------------------------------- */

#ifndef _ATE_TEST_FUNCTIONS_H
#define _ATE_TEST_FUNCTIONS_H

#define ANALOG_BIST_CLB_PASS        (0x5A5A5A5A)
#define ANALOG_BIST_CLB_MARKER_PTR  AMBA_FLASH_PTR->MANUFACTURER_ROW[0]
#define ANALOG_BIST_OSC_CALIB_PTR   AMBA_FLASH_PTR->FACTORY_ROW[0]
#define ANALOG_BIST_AFE_CALIB_PTR   AMBA_FLASH_PTR->FACTORY_ROW[1]

/* Calibrate analog */
int analog_bist(void);

/* Calibrate internal 32 kHz oscillator */
int Rc32kOscCal(void);

/* Calibrate internal 16 MHz oscillator */
int Rc16mOscCal(void);

/* Calibrate GNSSAFE Power Management */
int GnssAfePmCal(void);

#endif /* _ATE_TEST_FUNCTIONS_H */
