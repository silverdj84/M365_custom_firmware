/**
 * This file is part of the hoverboard-firmware-hack project.
 *
 * Copyright (C) 2020-2021 Emanuel FERU <aerdronix@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

// Includes
#include <stdlib.h> // for abs()
#include <string.h>
#include "stm32f1xx_hal.h"
#include "defines.h"
#include "config.h"
#include "cli_common.h"
#include "util.h"
#include "BLDC_controller.h"
#include "rtwtypes.h"

/* =========================== Variable Definitions =========================== */

//------------------------------------------------------------------------
// Global variables set externally
//------------------------------------------------------------------------

extern uint8_t enable;                  // global variable for motor enable

extern volatile uint32_t timeoutCnt; // Timeout counter for the General timeout (PPM, PWM, Nunchuck)

extern UART_HandleTypeDef huart1;

//------------------------------------------------------------------------
// Global variables set here in util.c
//------------------------------------------------------------------------
// Matlab defines - from auto-code generation
//---------------
RT_MODEL rtM_Left_; /* Real-time model */
RT_MODEL *const rtM_Left = &rtM_Left_;

extern P rtP_Left; /* Block parameters (auto storage) */
DW rtDW_Left; /* Observable states */
ExtU rtU_Left; /* External inputs */
ExtY rtY_Left; /* External outputs */

//---------------

int16_t cmd1;                          // normalized input value. -1000 to 1000
int16_t cmd2;                          // normalized input value. -1000 to 1000
int16_t input1;                        // Non normalized input value
int16_t input2;                        // Non normalized input value

int16_t speedAvg;                      // average measured speed
int16_t speedAvgAbs;                   // average measured speed in absolute
uint8_t timeoutFlagADC = 0; // Timeout Flag for ADC Protection:    0 = OK, 1 = Problem detected (line disconnected or wrong ADC data)
uint8_t timeoutFlagSerial = 0; // Timeout Flag for Rx Serial command: 0 = OK, 1 = Problem detected (line disconnected or wrong Rx data)

uint8_t ctrlModReqRaw = CTRL_MOD_REQ;
uint8_t ctrlModReq = CTRL_MOD_REQ;  // Final control mode request

//------------------------------------------------------------------------
// Local variables
//------------------------------------------------------------------------
static int16_t INPUT_MAX;             // [-] Input target maximum limitation
static int16_t INPUT_MIN;             // [-] Input target minimum limitation

/*
 #if !defined(VARIANT_HOVERBOARD) && !defined(VARIANT_TRANSPOTTER)
 static uint8_t  cur_spd_valid  = 0;
 static uint8_t  inp_cal_valid  = 0;
 static uint16_t INPUT1_TYP_CAL = INPUT1_TYPE;
 static uint16_t INPUT1_MIN_CAL = INPUT1_MIN;
 static uint16_t INPUT1_MID_CAL = INPUT1_MID;
 static uint16_t INPUT1_MAX_CAL = INPUT1_MAX;
 static uint16_t INPUT2_TYP_CAL = INPUT2_TYPE;
 static uint16_t INPUT2_MIN_CAL = INPUT2_MIN;
 static uint16_t INPUT2_MID_CAL = INPUT2_MID;
 static uint16_t INPUT2_MAX_CAL = INPUT2_MAX;
 #endif
 */

#if defined(CONTROL_SERIAL_USART2) || defined(CONTROL_SERIAL_USART3)
static SerialCommand command;
static SerialCommand command_raw;
static uint32_t command_len = sizeof(command);
#ifdef CONTROL_IBUS
  static uint16_t ibus_chksum;
  static uint16_t ibus_captured_value[IBUS_NUM_CHANNELS];
  #endif
#endif


#if defined(DEBUG_SERIAL_USART2) || defined(CONTROL_SERIAL_USART2) || defined(SIDEBOARD_SERIAL_USART2)
	static uint8_t rx_buffer_L[SERIAL_BUFFER_SIZE]; // USART Rx DMA circular buffer
	static uint32_t rx_buffer_L_len = SERIAL_BUFFER_SIZE; // ARRAY_LEN(rx_buffer_L);
#endif
#if defined(CONTROL_SERIAL_USART2) || defined(SIDEBOARD_SERIAL_USART2)
	static uint16_t timeoutCntSerial_L = 0; // Timeout counter for Rx Serial command
	static uint8_t timeoutFlagSerial_L = 0; // Timeout Flag for Rx Serial command: 0 = OK, 1 = Problem detected (line disconnected or wrong Rx data)
#endif


/* =========================== Initialization Functions =========================== */

void BLDC_Init(void) {
	/* Set BLDC controller parameters */
	/*
	 rtP_Left.z_selPhaCurMeasABC   = 1;            // Left motor measured current phases {Green, Blue} = {iA, iB} -> do NOT change
	 rtP_Left.z_ctrlTypSel         = CTRL_TYP_SEL;
	 rtP_Left.b_diagEna            = DIAG_ENA;
	 */

	rtP_Left.b_angleMeasEna = 0; // Motor angle input: 0 = estimated angle, 1 = measured angle (e.g. if encoder is available)
	rtP_Left.z_selPhaCurMeasABC = 0; // Left motor measured current phases {Green, Blue} = {iA, iB} -> do NOT change
	rtP_Left.z_ctrlTypSel = CTRL_TYP_SEL;
	rtP_Left.b_diagEna = DIAG_ENA;
	/*
	 rtP_Left.i_max                = (I_MOT_MAX * A2BIT_CONV) << 4;        // fixdt(1,16,4)
	 rtP_Left.n_max                = N_MOT_MAX << 4;                       // fixdt(1,16,4)
	 */
	rtP_Left.b_fieldWeakEna = FIELD_WEAK_ENA;
	rtP_Left.id_fieldWeakMax = (FIELD_WEAK_MAX * A2BIT_CONV) << 4; // fixdt(1,16,4)
	rtP_Left.a_phaAdvMax = PHASE_ADV_MAX << 4;                  // fixdt(1,16,4)
	rtP_Left.r_fieldWeakHi = FIELD_WEAK_HI << 4;                // fixdt(1,16,4)
	rtP_Left.r_fieldWeakLo = FIELD_WEAK_LO << 4;                // fixdt(1,16,4)

	/* Pack LEFT motor data into RTM */
	rtM_Left->defaultParam = &rtP_Left;
	rtM_Left->dwork = &rtDW_Left;
	rtM_Left->inputs = &rtU_Left;
	rtM_Left->outputs = &rtY_Left;

	/* Initialize BLDC controllers */
	BLDC_controller_initialize(rtM_Left);
}

void calcAvgSpeed(void) {
	// Calculate measured average speed. The minus sign (-) is because motors spin in opposite directions
#if   !defined(INVERT_L_DIRECTION) && !defined(INVERT_R_DIRECTION)
	speedAvg = rtY_Left.n_mot;
#elif !defined(INVERT_L_DIRECTION) &&  defined(INVERT_R_DIRECTION)
      speedAvg    = ( rtY_Left.n_mot + rtY_Right.n_mot) / 2;
    #elif  defined(INVERT_L_DIRECTION) && !defined(INVERT_R_DIRECTION)
      speedAvg    = (-rtY_Left.n_mot - rtY_Right.n_mot) / 2;
    #elif  defined(INVERT_L_DIRECTION) &&  defined(INVERT_R_DIRECTION)
      speedAvg    = (-rtY_Left.n_mot + rtY_Right.n_mot) / 2;
    #endif

	// Handle the case when SPEED_COEFFICIENT sign is negative (which is when most significant bit is 1)
	if (SPEED_COEFFICIENT & (1 << 16)) {
		speedAvg = -speedAvg;
	}
	speedAvgAbs = abs(speedAvg);
}

/*
 * Add Dead-band to a signal
 * This function realizes a dead-band around 0 and scales the input between [out_min, out_max]
 */
int addDeadBand(int16_t u, int16_t type, int16_t deadBand, int16_t in_min,
		int16_t in_mid, int16_t in_max, int16_t out_min, int16_t out_max) {
	switch (type) {
	case 0: // Input is ignored
		return 0;
	case 1: // Input is a normal pot
		return CLAMP(MAP(u, in_min, in_max, 0, out_max), 0, out_max);
	case 2: // Input is a mid resting pot
		if (u > in_mid - deadBand && u < in_mid + deadBand) {
			return 0;
		} else if (u > in_mid) {
			return CLAMP(MAP(u, in_mid + deadBand, in_max, 0, out_max), 0,
					out_max);
		} else {
			return CLAMP(MAP(u, in_mid - deadBand, in_min, 0, out_min), out_min,
					0);
		}
	default:
		return 0;
	}
}

/*
 * Electric Brake Function
 * In case of TORQUE mode, this function replaces the motor "freewheel" with a constant braking when the input torque request is 0.
 * This is useful when a small amount of motor braking is desired instead of "freewheel".
 *
 * Input: speedBlend = fixdt(0,16,15), reverseDir = {0, 1}
 * Output: cmd2 (Throtle) with brake component included
 */
void electricBrake(uint16_t speedBlend, uint8_t reverseDir) {
#if defined(ELECTRIC_BRAKE_ENABLE) && (CTRL_TYP_SEL == FOC_CTRL) && (CTRL_MOD_REQ == TRQ_MODE)
   int16_t brakeVal;

   // Make sure the Brake pedal is opposite to the direction of motion AND it goes to 0 as we reach standstill (to avoid Reverse driving)
   if (speedAvg > 0) {
     brakeVal = (int16_t)((-ELECTRIC_BRAKE_MAX * speedBlend) >> 15);
   } else {
     brakeVal = (int16_t)(( ELECTRIC_BRAKE_MAX * speedBlend) >> 15);
   }

   // Check if direction is reversed
   if (reverseDir) {
     brakeVal = -brakeVal;
   }

   // Calculate the new cmd2 with brake component included
   if (cmd2 >= 0 && cmd2 < ELECTRIC_BRAKE_THRES) {
     cmd2 = MAX(brakeVal, ((ELECTRIC_BRAKE_THRES - cmd2) * brakeVal) / ELECTRIC_BRAKE_THRES);
   } else if (cmd2 >= -ELECTRIC_BRAKE_THRES && cmd2 < 0) {
     cmd2 = MIN(brakeVal, ((ELECTRIC_BRAKE_THRES + cmd2) * brakeVal) / ELECTRIC_BRAKE_THRES);
   } else if (cmd2 >= ELECTRIC_BRAKE_THRES) {
     cmd2 = MAX(brakeVal, ((cmd2 - ELECTRIC_BRAKE_THRES) * INPUT_MAX) / (INPUT_MAX - ELECTRIC_BRAKE_THRES));
   } else {  // when (cmd2 < -ELECTRIC_BRAKE_THRES)
     cmd2 = MIN(brakeVal, ((cmd2 + ELECTRIC_BRAKE_THRES) * INPUT_MIN) / (INPUT_MIN + ELECTRIC_BRAKE_THRES));
   }
 #endif
}

/* =========================== Read Functions =========================== */

/*
 * Function to read the raw Input values from various input devices
 */
void readInput(void) {
#if defined(CONTROL_NUNCHUK) || defined(SUPPORT_NUNCHUK)
      if (nunchuk_connected != 0) {
        Nunchuk_Read();
        input1 = (nunchuk_data[0] - 127) * 8; // X axis 0-255
        input2 = (nunchuk_data[1] - 128) * 8; // Y axis 0-255
        #ifdef SUPPORT_BUTTONS
          button1 = (uint8_t)nunchuk_data[5] & 1;
          button2 = (uint8_t)(nunchuk_data[5] >> 1) & 1;
        #endif
      }
    #endif

#if defined(CONTROL_PPM_LEFT) || defined(CONTROL_PPM_RIGHT)
      input1 = (ppm_captured_value[0] - 500) * 2;
      input2 = (ppm_captured_value[1] - 500) * 2;
      #ifdef SUPPORT_BUTTONS
        button1 = ppm_captured_value[5] > 500;
        button2 = 0;
      #endif
    #endif

#if defined(CONTROL_PWM_LEFT) || defined(CONTROL_PWM_RIGHT)
      input1 = (pwm_captured_ch1_value - 500) * 2;
      input2 = (pwm_captured_ch2_value - 500) * 2;
    #endif

#ifdef CONTROL_ADC
      // ADC values range: 0-4095, see ADC-calibration in config.h
      input1 = adc_buffer.l_tx2;
      input2 = adc_buffer.l_rx2;
      timeoutCnt = 0;
    #endif

#if defined(CONTROL_SERIAL_USART2) || defined(CONTROL_SERIAL_USART3)
	// Handle received data validity, timeout and fix out-of-sync if necessary
#ifdef CONTROL_IBUS
        for (uint8_t i = 0; i < (IBUS_NUM_CHANNELS * 2); i+=2) {
          ibus_captured_value[(i/2)] = CLAMP(command.channels[i] + (command.channels[i+1] << 8) - 1000, 0, INPUT_MAX); // 1000-2000 -> 0-1000
        }
        input1 = (ibus_captured_value[0] - 500) * 2;
        input2 = (ibus_captured_value[1] - 500) * 2;
      #else
	input1 = command.steer;
	input2 = command.speed;
#endif
	timeoutCnt = 0;
#endif
}

/*
 * Function to calculate the command to the motors. This function also manages:
 * - timeout detection
 * - MIN/MAX limitations and deadband
 */
void readCommand(void) {
	readInput();
#ifdef CONTROL_ADC
      // If input1 or Input2 is either below MIN - Threshold or above MAX + Threshold, ADC protection timeout
      if (IN_RANGE(input1, (int16_t)INPUT1_MIN_CAL - ADC_PROTECT_THRESH, (int16_t)INPUT1_MAX_CAL + ADC_PROTECT_THRESH) &&
          IN_RANGE(input2, (int16_t)INPUT2_MIN_CAL - ADC_PROTECT_THRESH, (int16_t)INPUT2_MAX_CAL + ADC_PROTECT_THRESH)){
        if (timeoutFlagADC) {                           // Check for previous timeout flag
          if (timeoutCntADC-- <= 0)                     // Timeout de-qualification
            timeoutFlagADC  = 0;                        // Timeout flag cleared
        } else {
          timeoutCntADC     = 0;                        // Reset the timeout counter
        }
      } else {
        if (timeoutCntADC++ >= ADC_PROTECT_TIMEOUT) {   // Timeout qualification
          timeoutFlagADC    = 1;                        // Timeout detected
          timeoutCntADC     = ADC_PROTECT_TIMEOUT;      // Limit timout counter value
        }
      }
    #endif

#if defined(CONTROL_SERIAL_USART2) || defined(SIDEBOARD_SERIAL_USART2)
	if (timeoutCntSerial_L++ >= SERIAL_TIMEOUT) {     // Timeout qualification
		timeoutFlagSerial_L = 1;                        // Timeout detected
		timeoutCntSerial_L = SERIAL_TIMEOUT;       // Limit timout counter value
	}
	timeoutFlagSerial = timeoutFlagSerial_L;
#endif
#if defined(CONTROL_SERIAL_USART3) || defined(SIDEBOARD_SERIAL_USART3)
      if (timeoutCntSerial_R++ >= SERIAL_TIMEOUT) {     // Timeout qualification
        timeoutFlagSerial_R = 1;                        // Timeout detected
        timeoutCntSerial_R  = SERIAL_TIMEOUT;           // Limit timout counter value
      }
      timeoutFlagSerial = timeoutFlagSerial_R;
    #endif
#if defined(SIDEBOARD_SERIAL_USART2) && defined(SIDEBOARD_SERIAL_USART3)
      timeoutFlagSerial = timeoutFlagSerial_L || timeoutFlagSerial_R;
    #endif

	/*
	 #if !defined(VARIANT_HOVERBOARD) && !defined(VARIANT_TRANSPOTTER)
	 cmd1 = addDeadBand(input1, INPUT1_TYP_CAL, INPUT1_DEADBAND, INPUT1_MIN_CAL, INPUT1_MID_CAL, INPUT1_MAX_CAL, INPUT_MIN, INPUT_MAX);
	 #if !defined(VARIANT_SKATEBOARD)
	 cmd2 = addDeadBand(input2, INPUT2_TYP_CAL, INPUT2_DEADBAND, INPUT2_MIN_CAL, INPUT2_MID_CAL, INPUT2_MAX_CAL, INPUT_MIN, INPUT_MAX);
	 #else
	 cmd2 = addDeadBand(input2, INPUT2_TYP_CAL, INPUT2_DEADBAND, INPUT2_MIN_CAL, INPUT2_MID_CAL, INPUT2_MAX_CAL, INPUT2_BRAKE, INPUT_MAX);
	 #endif
	 #endif
	 */

#ifdef VARIANT_TRANSPOTTER
      #ifdef GAMETRAK_CONNECTION_NORMAL
        cmd1 = adc_buffer.l_rx2;
        cmd2 = adc_buffer.l_tx2;
      #endif
      #ifdef GAMETRAK_CONNECTION_ALTERNATE
        cmd1 = adc_buffer.l_tx2;
        cmd2 = adc_buffer.l_rx2;
      #endif
    #endif

#ifdef VARIANT_HOVERCAR
      brakePressed = (uint8_t)(cmd1 > 50);
    #endif

	if (timeoutFlagADC || timeoutFlagSerial || timeoutCnt > TIMEOUT) { // In case of timeout bring the system to a Safe State
		ctrlModReq = OPEN_MODE; // Request OPEN_MODE. This will bring the motor power to 0 in a controlled way
		cmd1 = 0;
		cmd2 = 0;
	} else {
		ctrlModReq = ctrlModReqRaw;                   // Follow the Mode request
	}

#if defined(SUPPORT_BUTTONS_LEFT) || defined(SUPPORT_BUTTONS_RIGHT)
      button1 = !HAL_GPIO_ReadPin(BUTTON1_PORT, BUTTON1_PIN);
      button2 = !HAL_GPIO_ReadPin(BUTTON2_PORT, BUTTON2_PIN);
    #endif

#if defined(CRUISE_CONTROL_SUPPORT) && (defined(SUPPORT_BUTTONS) || defined(SUPPORT_BUTTONS_LEFT) || defined(SUPPORT_BUTTONS_RIGHT))
      cruiseControl(button1);                                           // Cruise control activation/deactivation
    #endif
}

/*
 * Check for new data received on USART2 with DMA: refactored function from https://github.com/MaJerle/stm32-usart-uart-dma-rx-tx
 * - this function is called for every USART IDLE line detection, in the USART interrupt handler
 */
void usart2_rx_check(void) {
#if defined(DEBUG_SERIAL_USART2) || defined(CONTROL_SERIAL_USART2) || defined(SIDEBOARD_SERIAL_USART2)
	static uint32_t old_pos;
	uint32_t pos;
	pos = rx_buffer_L_len - __HAL_DMA_GET_COUNTER(huart1.hdmarx); // Calculate current position in buffer
#endif

#if defined(DEBUG_SERIAL_USART2)
  if (pos != old_pos) {                                                 // Check change in received data
    if (pos > old_pos) {                                                // "Linear" buffer mode: check if current position is over previous one
      usart_process_debug(&rx_buffer_L[old_pos], pos - old_pos);        // Process data
    } else {                                                            // "Overflow" buffer mode
      usart_process_debug(&rx_buffer_L[old_pos], rx_buffer_L_len - old_pos); // First Process data from the end of buffer
      if (pos > 0) {                                                    // Check and continue with beginning of buffer
        usart_process_debug(&rx_buffer_L[0], pos);                      // Process remaining data
      }
    }
  }
  #endif // DEBUG_SERIAL_USART2

#ifdef CONTROL_SERIAL_USART2
	uint8_t *ptr;
	if (pos != old_pos) {                       // Check change in received data
		ptr = (uint8_t*) &command_raw; // Initialize the pointer with command_raw address
		if (pos > old_pos && (pos - old_pos) == command_len) { // "Linear" buffer mode: check if current position is over previous one AND data length equals expected length
			memcpy(ptr, &rx_buffer_L[old_pos], command_len); // Copy data. This is possible only if command_raw is contiguous! (meaning all the structure members have the same size)
			usart_process_command(&command_raw, &command, 2);    // Process data
		} else if ((rx_buffer_L_len - old_pos + pos) == command_len) { // "Overflow" buffer mode: check if data length equals expected length
			memcpy(ptr, &rx_buffer_L[old_pos], rx_buffer_L_len - old_pos); // First copy data from the end of buffer
			if (pos > 0) {        // Check and continue with beginning of buffer
				ptr += rx_buffer_L_len - old_pos; // Move to correct position in command_raw
				memcpy(ptr, &rx_buffer_L[0], pos);        // Copy remaining data
			}
			usart_process_command(&command_raw, &command, 2);    // Process data
		}
	}
#endif // CONTROL_SERIAL_USART2

#ifdef SIDEBOARD_SERIAL_USART2
  uint8_t *ptr;
  if (pos != old_pos) {                                                 // Check change in received data
    ptr = (uint8_t *)&Sideboard_L_raw;                                  // Initialize the pointer with Sideboard_raw address
    if (pos > old_pos && (pos - old_pos) == Sideboard_L_len) {          // "Linear" buffer mode: check if current position is over previous one AND data length equals expected length
      memcpy(ptr, &rx_buffer_L[old_pos], Sideboard_L_len);              // Copy data. This is possible only if Sideboard_raw is contiguous! (meaning all the structure members have the same size)
      usart_process_sideboard(&Sideboard_L_raw, &Sideboard_L, 2);       // Process data
    } else if ((rx_buffer_L_len - old_pos + pos) == Sideboard_L_len) {  // "Overflow" buffer mode: check if data length equals expected length
      memcpy(ptr, &rx_buffer_L[old_pos], rx_buffer_L_len - old_pos);    // First copy data from the end of buffer
      if (pos > 0) {                                                    // Check and continue with beginning of buffer
        ptr += rx_buffer_L_len - old_pos;                               // Move to correct position in Sideboard_raw
        memcpy(ptr, &rx_buffer_L[0], pos);                              // Copy remaining data
      }
      usart_process_sideboard(&Sideboard_L_raw, &Sideboard_L, 2);       // Process data
    }
  }
  #endif // SIDEBOARD_SERIAL_USART2

#if defined(DEBUG_SERIAL_USART2) || defined(CONTROL_SERIAL_USART2) || defined(SIDEBOARD_SERIAL_USART2)
	old_pos = pos;                                        // Update old position
	if (old_pos == rx_buffer_L_len) { // Check and manually update if we reached end of buffer
		old_pos = 0;
	}
#endif
}

/*
 * Check for new data received on USART3 with DMA: refactored function from https://github.com/MaJerle/stm32-usart-uart-dma-rx-tx
 * - this function is called for every USART IDLE line detection, in the USART interrupt handler
 */
void usart3_rx_check(void) {
#if defined(DEBUG_SERIAL_USART3) || defined(CONTROL_SERIAL_USART3) || defined(SIDEBOARD_SERIAL_USART3)
  static uint32_t old_pos;
  uint32_t pos;
  pos = rx_buffer_R_len - __HAL_DMA_GET_COUNTER(huart3.hdmarx);         // Calculate current position in buffer
  #endif

#if defined(DEBUG_SERIAL_USART3)
  if (pos != old_pos) {                                                 // Check change in received data
    if (pos > old_pos) {                                                // "Linear" buffer mode: check if current position is over previous one
      usart_process_debug(&rx_buffer_R[old_pos], pos - old_pos);        // Process data
    } else {                                                            // "Overflow" buffer mode
      usart_process_debug(&rx_buffer_R[old_pos], rx_buffer_R_len - old_pos); // First Process data from the end of buffer
      if (pos > 0) {                                                    // Check and continue with beginning of buffer
        usart_process_debug(&rx_buffer_R[0], pos);                      // Process remaining data
      }
    }
  }
  #endif // DEBUG_SERIAL_USART3

#ifdef CONTROL_SERIAL_USART3
  uint8_t *ptr;
  if (pos != old_pos) {                                                 // Check change in received data
    ptr = (uint8_t *)&command_raw;                                      // Initialize the pointer with command_raw address
    if (pos > old_pos && (pos - old_pos) == command_len) {              // "Linear" buffer mode: check if current position is over previous one AND data length equals expected length
      memcpy(ptr, &rx_buffer_R[old_pos], command_len);                  // Copy data. This is possible only if command_raw is contiguous! (meaning all the structure members have the same size)
      usart_process_command(&command_raw, &command, 3);                 // Process data
    } else if ((rx_buffer_R_len - old_pos + pos) == command_len) {      // "Overflow" buffer mode: check if data length equals expected length
      memcpy(ptr, &rx_buffer_R[old_pos], rx_buffer_R_len - old_pos);    // First copy data from the end of buffer
      if (pos > 0) {                                                    // Check and continue with beginning of buffer
        ptr += rx_buffer_R_len - old_pos;                               // Move to correct position in command_raw
        memcpy(ptr, &rx_buffer_R[0], pos);                              // Copy remaining data
      }
      usart_process_command(&command_raw, &command, 3);                 // Process data
    }
  }
  #endif // CONTROL_SERIAL_USART3

#ifdef SIDEBOARD_SERIAL_USART3
  uint8_t *ptr;
  if (pos != old_pos) {                                                 // Check change in received data
    ptr = (uint8_t *)&Sideboard_R_raw;                                  // Initialize the pointer with Sideboard_raw address
    if (pos > old_pos && (pos - old_pos) == Sideboard_R_len) {          // "Linear" buffer mode: check if current position is over previous one AND data length equals expected length
      memcpy(ptr, &rx_buffer_R[old_pos], Sideboard_R_len);              // Copy data. This is possible only if Sideboard_raw is contiguous! (meaning all the structure members have the same size)
      usart_process_sideboard(&Sideboard_R_raw, &Sideboard_R, 3);       // Process data
    } else if ((rx_buffer_R_len - old_pos + pos) == Sideboard_R_len) {  // "Overflow" buffer mode: check if data length equals expected length
      memcpy(ptr, &rx_buffer_R[old_pos], rx_buffer_R_len - old_pos);    // First copy data from the end of buffer
      if (pos > 0) {                                                    // Check and continue with beginning of buffer
        ptr += rx_buffer_R_len - old_pos;                               // Move to correct position in Sideboard_raw
        memcpy(ptr, &rx_buffer_R[0], pos);                              // Copy remaining data
      }
      usart_process_sideboard(&Sideboard_R_raw, &Sideboard_R, 3);       // Process data
    }
  }
  #endif // SIDEBOARD_SERIAL_USART3

#if defined(DEBUG_SERIAL_USART3) || defined(CONTROL_SERIAL_USART3) || defined(SIDEBOARD_SERIAL_USART3)
  old_pos = pos;                                                        // Update old position
  if (old_pos == rx_buffer_R_len) {                                     // Check and manually update if we reached end of buffer
    old_pos = 0;
  }
  #endif
}

/*
 * Process Rx debug user command input
 */
#if defined(DEBUG_SERIAL_USART2) || defined(DEBUG_SERIAL_USART3)
void usart_process_debug(uint8_t *userCommand, uint32_t len)
{
  for (; len > 0; len--, userCommand++) {
    if (*userCommand != '\n' && *userCommand != '\r') {   // Do not accept 'new line' and 'carriage return' commands
      consoleLog("-- Command received --\r\n");
      // handle_input(*userCommand);                      // -> Create this function to handle the user commands
    }
  }
}
#endif // SERIAL_DEBUG

/*
 * Process command Rx data
 * - if the command_in data is valid (correct START_FRAME and checksum) copy the command_in to command_out
 */
#if defined(CONTROL_SERIAL_USART2) || defined(CONTROL_SERIAL_USART3)
void usart_process_command(SerialCommand *command_in,
		SerialCommand *command_out, uint8_t usart_idx) {
#ifdef CONTROL_IBUS
    if (command_in->start == IBUS_LENGTH && command_in->type == IBUS_COMMAND) {
      ibus_chksum = 0xFFFF - IBUS_LENGTH - IBUS_COMMAND;
      for (uint8_t i = 0; i < (IBUS_NUM_CHANNELS * 2); i++) {
        ibus_chksum -= command_in->channels[i];
      }
      if (ibus_chksum == (uint16_t)((command_in->checksumh << 8) + command_in->checksuml)) {
        *command_out = *command_in;
        if (usart_idx == 2) {             // Sideboard USART2
          #ifdef CONTROL_SERIAL_USART2
          timeoutCntSerial_L  = 0;        // Reset timeout counter
          timeoutFlagSerial_L = 0;        // Clear timeout flag
          #endif
        } else if (usart_idx == 3) {      // Sideboard USART3
          #ifdef CONTROL_SERIAL_USART3
          timeoutCntSerial_R  = 0;        // Reset timeout counter
          timeoutFlagSerial_R = 0;        // Clear timeout flag
          #endif
        }
      }
    }
  #else
	uint16_t checksum;
	if (command_in->start == SERIAL_START_FRAME) {
		checksum = (uint16_t) (command_in->start ^ command_in->steer
				^ command_in->speed);
		if (command_in->checksum == checksum) {
			*command_out = *command_in;
			if (usart_idx == 2) {             // Sideboard USART2
#ifdef CONTROL_SERIAL_USART2
				timeoutCntSerial_L = 0;        // Reset timeout counter
				timeoutFlagSerial_L = 0;        // Clear timeout flag
#endif
			} else if (usart_idx == 3) {      // Sideboard USART3
#ifdef CONTROL_SERIAL_USART3
        timeoutCntSerial_R  = 0;        // Reset timeout counter
        timeoutFlagSerial_R = 0;        // Clear timeout flag
        #endif
			}
		}
	}
#endif
}
#endif

/*
 * Process Sideboard Rx data
 * - if the Sideboard_in data is valid (correct START_FRAME and checksum) copy the Sideboard_in to Sideboard_out
 */
#if defined(SIDEBOARD_SERIAL_USART2) || defined(SIDEBOARD_SERIAL_USART3)
void usart_process_sideboard(SerialSideboard *Sideboard_in, SerialSideboard *Sideboard_out, uint8_t usart_idx)
{
  uint16_t checksum;
  if (Sideboard_in->start == SERIAL_START_FRAME) {
    checksum = (uint16_t)(Sideboard_in->start ^ Sideboard_in->pitch ^ Sideboard_in->dPitch ^ Sideboard_in->cmd1 ^ Sideboard_in->cmd2 ^ Sideboard_in->sensors);
    if (Sideboard_in->checksum == checksum) {
      *Sideboard_out = *Sideboard_in;
      if (usart_idx == 2) {             // Sideboard USART2
        #ifdef SIDEBOARD_SERIAL_USART2
        timeoutCntSerial_L  = 0;        // Reset timeout counter
        timeoutFlagSerial_L = 0;        // Clear timeout flag
        #endif
      } else if (usart_idx == 3) {      // Sideboard USART3
        #ifdef SIDEBOARD_SERIAL_USART3
        timeoutCntSerial_R  = 0;        // Reset timeout counter
        timeoutFlagSerial_R = 0;        // Clear timeout flag
        #endif
      }
    }
  }
}
#endif

/* =========================== Poweroff Functions =========================== */

void poweroff(void) {
	enable = 0;
	//consoleLog("-- Motors disabled --\r\n");
	/*
	 buzzerCount = 0;  // prevent interraction with beep counter
	 buzzerPattern = 0;
	 for (int i = 0; i < 8; i++) {
	 buzzerFreq = (uint8_t)i;
	 HAL_Delay(100);
	 }
	 saveConfig();
	 HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, GPIO_PIN_RESET);

	 */
	while (1) {
	}
}

void poweroffPressCheck(void) {
#if !defined(VARIANT_HOVERBOARD) && !defined(VARIANT_TRANSPOTTER)
	if (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) {
		enable = 0;
		/*
		uint16_t cnt_press = 0;
		 while(HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) {
		 HAL_Delay(10);
		 if (cnt_press++ == 5 * 100) { beepShort(5); }
		 }
		 if (cnt_press >= 5 * 100) {                         // Check if press is more than 5 sec
		 HAL_Delay(1000);
		 if (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) {  // Double press: Adjust Max Current, Max Speed
		 while(HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) { HAL_Delay(10); }
		 beepLong(8);
		 updateCurSpdLim();
		 beepShort(5);
		 } else {                                          // Long press: Calibrate ADC Limits
		 beepLong(16);
		 adcCalibLim();
		 beepShort(5);
		 }
		 } else {                                            // Short press: power off
		 poweroff();
		 }
		 */                                 // Short press: power off
		poweroff();
	}
#elif defined(VARIANT_TRANSPOTTER)
    if(HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) {
      enable = 0;
      while(HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) { HAL_Delay(10); }
      beepShort(5);
      HAL_Delay(300);
      if (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) {
        while(HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) { HAL_Delay(10); }
        beepLong(5);
        HAL_Delay(350);
        poweroff();
      } else {
        setDistance += 0.25;
        if (setDistance > 2.6) {
          setDistance = 0.5;
        }
        beepShort(setDistance / 0.25);
        saveValue = setDistance * 1000;
        saveValue_valid = 1;
      }
    }
  #else
    if (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) {
      enable = 0;                                             // disable motors
      while (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) {}    // wait until button is released
      poweroff();                                             // release power-latch
    }
  #endif
}

/* =========================== Filtering Functions =========================== */

/* Low pass filter fixed-point 32 bits: fixdt(1,32,20)
 * Max:  2047.9375
 * Min: -2048
 * Res:  0.0625
 *
 * Inputs:       u     = int16 or int32
 * Outputs:      y     = fixdt(1,32,16)
 * Parameters:   coef  = fixdt(0,16,16) = [0,65535U]
 *
 * Example:
 * If coef = 0.8 (in floating point), then coef = 0.8 * 2^16 = 52429 (in fixed-point)
 * filtLowPass16(u, 52429, &y);
 * yint = (int16_t)(y >> 16); // the integer output is the fixed-point ouput shifted by 16 bits
 */
void filtLowPass32(int32_t u, uint16_t coef, int32_t *y) {
	int64_t tmp;
	tmp = ((int64_t) ((u << 4) - (*y >> 12)) * coef) >> 4;
	tmp = CLAMP(tmp, -2147483648LL, 2147483647LL); // Overflow protection: 2147483647LL = 2^31 - 1
	*y = (int32_t) tmp + (*y);
}

/* rateLimiter16(int16_t u, int16_t rate, int16_t *y);
 * Inputs:       u     = int16
 * Outputs:      y     = fixdt(1,16,4)
 * Parameters:   rate  = fixdt(1,16,4) = [0, 32767] Do NOT make rate negative (>32767)
 */
void rateLimiter16(int16_t u, int16_t rate, int16_t *y) {
	int16_t q0;
	int16_t q1;

	q0 = (u << 4) - *y;

	if (q0 > rate) {
		q0 = rate;
	} else {
		q1 = -rate;
		if (q0 < q1) {
			q0 = q1;
		}
	}

	*y = q0 + *y;
}

/* mixerFcn(rtu_speed, rtu_steer, &rty_speedR, &rty_speedL);
 * Inputs:       rtu_speed, rtu_steer                  = fixdt(1,16,4)
 * Outputs:      rty_speedR, rty_speedL                = int16_t
 * Parameters:   SPEED_COEFFICIENT, STEER_COEFFICIENT  = fixdt(0,16,14)
 */
void mixerFcn(int16_t rtu_speed, int16_t rtu_steer, int16_t *rty_speedR,
		int16_t *rty_speedL) {
	int16_t prodSpeed;
	int16_t prodSteer;
	int32_t tmp;

	prodSpeed = (int16_t) ((rtu_speed * (int16_t) SPEED_COEFFICIENT) >> 14);
	prodSteer = (int16_t) ((rtu_steer * (int16_t) STEER_COEFFICIENT) >> 14);

	tmp = prodSpeed - prodSteer;
	tmp = CLAMP(tmp, -32768, 32767);  // Overflow protection
	*rty_speedR = (int16_t) (tmp >> 4);       // Convert from fixed-point to int
	*rty_speedR = CLAMP(*rty_speedR, INPUT_MIN, INPUT_MAX);

	tmp = prodSpeed + prodSteer;
	tmp = CLAMP(tmp, -32768, 32767);  // Overflow protection
	*rty_speedL = (int16_t) (tmp >> 4);       // Convert from fixed-point to int
	*rty_speedL = CLAMP(*rty_speedL, INPUT_MIN, INPUT_MAX);
}

