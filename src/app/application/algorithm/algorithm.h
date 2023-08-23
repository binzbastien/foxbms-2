/**
 *
 * @copyright &copy; 2010 - 2022, Fraunhofer-Gesellschaft zur Foerderung der angewandten Forschung e.V.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * We kindly request you to use one or more of the following phrases to refer to
 * foxBMS in your hardware, software, documentation or advertising materials:
 *
 * - &Prime;This product uses parts of foxBMS&reg;&Prime;
 * - &Prime;This product includes parts of foxBMS&reg;&Prime;
 * - &Prime;This product is derived from foxBMS&reg;&Prime;
 *
 */

/**
 * @file    algorithm.h
 * @author  foxBMS Team
 * @date    2017-12-18 (date of creation)
 * @updated 2022-10-27 (date of last update)
 * @version v1.4.1
 * @ingroup ALGORITHMS
 * @prefix  ALGO
 *
 * @brief   Headers for the driver for the storage in the EEPROM memory
 *
 * @details Header file driver of external EEPROM device
 *
 */

#ifndef FOXBMS__ALGORITHM_H_
#define FOXBMS__ALGORITHM_H_
#define M 3
#define N 3
/*========== Includes =======================================================*/
#include "algorithm_cfg.h"

/*========== Macros and Definitions =========================================*/

/*========== Extern Constant and Variable Declarations ======================*/

/*========== Extern Function Prototypes =====================================*/
/**
 * @brief   Calling this function sets a signal that lets
 *          #ALGO_Initialization() know that the initialization has to be run.
 */
extern void ALGO_UnlockInitialization(void);

/**
 * @brief   handles the call of different algorithm functions when cycle time
 *          has expired
 */
extern void ALGO_MainFunction(void);

/**
 * @brief   monitors the calculation duration of the different algorithms
 */
extern void ALGO_MonitorExecutionTime(void);

/**
 * @brief  SOC prediction algorithm for NMC and LFP chemistry, SOH estimation for the NMC only
 * @param chemistry NMC =>0 | LFP =>1
 * @param temperature_cell Temperature of the cell in degrees
 * @param Current_mA Current flows through the cell
 * @param voltage_cell1 Cell voltage measurement
 */
extern void ALGO_bfh_SOC_NMC_LFP(bool chemistry, int16_t temperature_cell, int16_t Current_mA, int16_t voltage_cell1);

/**
 * @brief  Select the ECM parameters in function of the chemistry, the temperature and the SOC
 * @param chemistry NMC =>0 | LFP =>1
 * @param temperature_cell Temperature of the cell in degrees
 * @param voltage_cell1 Cell voltage measurement
 */
extern void Parameters_selection(bool chemistry, int16_t temperature_cell, float SOC);
void jacobi(float A[M][N], float U[M][M], float S[N], float VT[N][N]);
/*========== Externalized Static Functions Prototypes (Unit Test) ===========*/
#ifdef UNITY_UNIT_TEST
extern void TEST_ALGO_ResetInitializationRequest(void);
#endif /* UNITY_UNIT_TEST */
#endif /* FOXBMS__ALGORITHM_H_ */
