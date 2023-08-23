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
 * THystLookupTableIS SOFTWARE IS PROVIDED BY THystLookupTableE COPYRIGHystLookupTableT HystLookupTableOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THystLookupTableE
 * IMPLIED WARRANTIES OF MERCHystLookupTableANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHystLookupTableALL THystLookupTableE COPYRIGHystLookupTableT HystLookupTableOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HystLookupTableOWEVER
 * CAUSED AND ON ANY THystLookupTableEORY OF LIABILITY, WHystLookupTableETHystLookupTableER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHystLookupTableERWISE) ARISING IN ANY WAY OUT OF THystLookupTableE USE
 * OF THystLookupTableIS SOFTWARE, EVEN IF ADVISED OF THystLookupTableE POSSIBILITY OF SUCHystLookupTable DAMAGE.
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
 * @file    algorithm.c
 * @author  foxBMS Team
 * @date    2017-12-18 (date of creation)
 * @updated 2022-10-27 (date of last update)
 * @version v1.4.1
 * @ingroup ALGORITHystLookupTableMS
 * @prefix  ALGO
 *
 * @brief   Main module to handle the execution of algorithms
 *
 *
 */

/*========== Includes =======================================================*/
#include "algorithm.h"

#include "battery_cell_cfg.h"

#include "database.h"
#include "os.h"

#include <math.h>
#include <stdio.h>
/*========== Macros and Definitions =========================================*/

/*========== Static Constant and Variable Definitions =======================*/
/**
 * This is a signal that skips initialization of #ALGO_Initialization()
 * until it has been requested.
 */
static bool algo_initializationRequested = false;

/*========== Extern Constant and Variable Definitions =======================*/

/*========== Static Function Prototypes =====================================*/
/**
 * @brief   initializes local variables and module internals needed to use the
 *          algorithm module
 */
static void ALGO_Initialization(void);

/*========== Static Function Implementations ================================*/
static void ALGO_Initialization(void) {
    /* iterate over all algorithms */
    for (uint16_t i = 0u; i < algo_length; i++) {
        /* check if the cycle time is valid */
        FAS_ASSERT((algo_algorithms[i].cycleTime_ms % ALGO_TICK_ms) == 0u);

        /* check only uninitialized algorithms */
        if (algo_algorithms[i].state == ALGO_UNINITIALIZED) {
            /* directly make ready when init function is a null pointer otherwise run init */
            if (algo_algorithms[i].fpInitialization == NULL_PTR) {
                algo_algorithms[i].state = ALGO_READY;
            } else {
                const STD_RETURN_TYPE_e result = algo_algorithms[i].fpInitialization();
                FAS_ASSERT((STD_OK == result) || (STD_NOT_OK == result));
                if (STD_OK == result) {
                    algo_algorithms[i].state = ALGO_READY;
                } else {
                    algo_algorithms[i].state = ALGO_FAILED_INIT;
                }
            }
        }
    }
}

/*========== Extern Function Implementations ================================*/

extern void ALGO_UnlockInitialization(void) {
    OS_EnterTaskCritical();
    algo_initializationRequested = true;
    OS_ExitTaskCritical();
}

extern void ALGO_MainFunction(void) {
    OS_EnterTaskCritical();
    const bool initializationRequested = algo_initializationRequested;
    OS_ExitTaskCritical();
    if (initializationRequested == true) {
        ALGO_Initialization();
        OS_EnterTaskCritical();
        algo_initializationRequested = false;
        OS_ExitTaskCritical();
    }

    static uint32_t counter_ticks = 0u;

    for (uint16_t i = 0u; i < algo_length; i++) {
        const bool runAlgorithmAsap = (algo_algorithms[i].cycleTime_ms == 0u);
        const bool runAlgorithmCycleElapsed =
            ((algo_algorithms[i].cycleTime_ms != 0u) && ((counter_ticks % algo_algorithms[i].cycleTime_ms) == 0u));
        if ((runAlgorithmAsap != false) || (runAlgorithmCycleElapsed != false)) {
            /* Cycle time elapsed -> call function */
            if (algo_algorithms[i].state == ALGO_READY) {
                /* Set state to running -> reset to READY before leaving algo function */
                algo_algorithms[i].state     = ALGO_RUNNING;
                algo_algorithms[i].startTime = OS_GetTickCount();
                algo_algorithms[i].fpAlgorithm();
                ALGO_MarkAsDone(i);
            }
            /* check if we need to reinit */
            if (algo_algorithms[i].state == ALGO_REINIT_REQUESTED) {
                /* set to uninitialized so that the algorithm can be reinitialized */
                algo_algorithms[i].state = ALGO_UNINITIALIZED;

                ALGO_UnlockInitialization();
            }
        }
    }

    counter_ticks += ALGO_TICK_ms;
}

extern void ALGO_MonitorExecutionTime(void) {
    const uint32_t timestamp = OS_GetTickCount();

    for (uint16_t i = 0u; i < algo_length; i++) {
        if ((algo_algorithms[i].startTime != 0u) && (algo_algorithms[i].state == ALGO_RUNNING) &&
            ((algo_algorithms[i].startTime + algo_algorithms[i].maxCalculationDuration_ms) < timestamp)) {
            /* Block task from further execution because of runtime violation, but task will finish its execution */
            algo_algorithms[i].state = ALGO_BLOCKED;

            /* TODO: Add diag call to notify error in algorithm module */
        }
    }
}

extern void ALGO_bfh_SOC_NMC_LFP(bool chemistry, int16_t temperature_cell, int16_t Current_mA, int16_t voltage_cell1) {
    // Read the data
    DATA_BLOCK_EKF_s data_blockEKF = {.header.uniqueId = DATA_BLOCK_ID_EKF};
    DATA_READ_DATA(&data_blockEKF);
    DATA_BLOCK_ECM_s data_blockECM = {.header.uniqueId = DATA_BLOCK_ID_ECM};
    DATA_READ_DATA(&data_blockECM);
    //-------------------------------------------------------------------------
    // Simplify the name of the variable
    float ik = ((float)Current_mA) / 1000;  // Conversion in [A]
    if (fabs(ik) < 0.04) {
        ik = 0;
    }
    float delat_t = 0.9882;
    float vk      = (float)voltage_cell1 / 1000;  // Conversion in [V]
    float SOC     = data_blockECM.SOC_EKF;        //The value is in [-]
    float SOH     = data_blockECM.SOH;            //SOH value
    float xhat_next[3];
    float Ucell;                                                 // Voltage of the cell
    float R0_SOH  = data_blockECM.R0 * (1 + (1 - SOH) * 0.075);  // Internal resistor
    float R1_SOH  = data_blockECM.R1 * (1 + (1 - SOH) * 2.5);    // R1 value including SOH
    float R2_SOH  = data_blockECM.R2 * (1 + (1 - SOH) * 11);     // R2 value including SOH
    float OCV_SOH = data_blockECM.OCV + (1.3757e-6 * pow(SOC, 4) + 1.4129e-05 * pow(SOC, 2) - 0.0357 * pow(SOC, 2) +
                                         1.7142 * SOC + (1 - SOH) * 250) /
                                            1000;  // R2 value including SOH
    //-------------------------------------------------------------------------
    // 2. Init the SigmaX, SigmaV, SigmaW and xhat
    if (data_blockEKF.SigmaV == 0) {
        data_blockEKF.SigmaX[0][0] = 1e-3;  // uncertainty of initial state of URC1
        data_blockEKF.SigmaX[1][1] = 1e-3;  // uncertainty of initial state of URC2
        data_blockEKF.SigmaX[2][2] = 1e-3;  // uncertainty of initial state of SOC estimated
        data_blockEKF.SigmaV       = 2e-1;  // Uncertainty of voltage sensor, output equation
        data_blockEKF.SigmaW       = 2e-1;  // Uncertainty of current sensor, state equation

        data_blockEKF.xhat[0] = 0;    // Initial value of Urc1
        data_blockEKF.xhat[1] = 0;    // Initial value of Urc2
        data_blockEKF.xhat[2] = SOC;  // Initial value of SOC estimated
    }
    //-------------------------------------------------------------------------
    // 3. LFP : Current sensibility of the hysteresis
    if (chemistry) {
        if (signbit(data_blockECM.Current_mA_old) != signbit(ik)) {
            data_blockECM.U_hyst_old = data_blockECM.U_hyst;
            data_blockECM.time_hyst  = 1;
        } else {
            data_blockECM.time_hyst++;
        }
        if (ik == 0) {
            data_blockECM.U_hyst_new = data_blockECM.U_hyst_new * 0.5;
        }
        data_blockECM.U_hyst = data_blockECM.U_hyst_old + (data_blockECM.U_hyst_new - data_blockECM.U_hyst_old) *
                                                              (1 - exp(-data_blockECM.time_hyst / 300));
        if (data_blockECM.time_hyst >= 1500) {
            data_blockECM.U_hyst_old = data_blockECM.U_hyst;
            data_blockECM.U_hyst     = data_blockECM.U_hyst_new;
        }
    }

    // EKF Step 0 : Compute the Ahat and Bhat matrix
    // Ahat : Jacobian matrix of the partial derivatives
    data_blockEKF.Ahat[0][0] = exp(-delat_t / data_blockECM.Tau1);  //partial derivative of Urc1
    data_blockEKF.Ahat[1][1] = exp(-delat_t / data_blockECM.Tau2);  //partial derivative of Urc2
    data_blockEKF.Ahat[2][2] = 1;                                   //partial derivative of SOC
    // Bhat : Input matrix which represent the offset of the model
    data_blockEKF.Bhat[0] = R1_SOH * (1 - data_blockEKF.Ahat[0][0]);  //partial derivative of Urc1
    data_blockEKF.Bhat[1] = R2_SOH * (1 - data_blockEKF.Ahat[1][1]);  //partial derivative of Urc2
    data_blockEKF.Bhat[2] = (float)-delat_t / (3600 * Qcell * SOH);   //partial derivative of SOC
    //-------------------------------------------------------------------------
    // EKF Step 1a : Project the State Ahead
    // xhat = Ahat*xhat + B*ik;
    for (int row = 0; row < 3; row++) {
        for (int col = 0; col < 3; col++) {
            xhat_next[col] = data_blockEKF.Ahat[row][col] * data_blockEKF.xhat[row];
        }
        data_blockEKF.xhat[row] = xhat_next[0] + xhat_next[1] + xhat_next[2] + data_blockEKF.Bhat[row] * ik;
    }
    // Boundaries conditions
    data_blockEKF.xhat[0] = fmaxf(fminf(data_blockEKF.xhat[0], 0.02f), -0.02f);
    data_blockEKF.xhat[1] = fmaxf(fminf(data_blockEKF.xhat[1], 0.02f), -0.02f);
    data_blockEKF.xhat[2] = fmaxf(fminf(data_blockEKF.xhat[2], 1.05f), 0.0);
    // ------------------------------------------------------------------------
    // EKF Step 1b : Update SigmaX
    float Ahat_Sigmax[3][3];       //Used only here to update SigmaX
    float Ahat_Sigmax_Ahat[3][3];  //Used only here to update SigmaX
    for (int g = 0; g < 3; g++) {
        for (int h = 0; h < 3; h++) {
            for (int i = 0; i < 3; i++) {
                if (g < 2) {
                    for (int j = 0; j < 3; j++) {
                        if (g == 0)
                            Ahat_Sigmax[h][i] = data_blockEKF.Ahat[h][j] * data_blockEKF.SigmaX[j][i];
                        else
                            Ahat_Sigmax_Ahat[h][i] = Ahat_Sigmax[h][j] * data_blockEKF.Ahat[i][j];
                    }
                } else
                    data_blockEKF.SigmaX[h][i] = Ahat_Sigmax_Ahat[h][i] +
                                                 data_blockEKF.Bhat[i] * data_blockEKF.SigmaW * data_blockEKF.Bhat[h];
            }
        }
    }
    // ------------------------------------------------------------------------
    // EKF Step 1c : Ucell estimation
    float Uro = (R0_SOH * ik);  // Internal voltage losses
    Ucell     = data_blockECM.OCV - Uro - data_blockEKF.xhat[0] - data_blockEKF.xhat[1] -
            signbit(ik) * data_blockECM.U_hyst;
    float error_voltage = 0;
    /*NMC correction factor*/
    if (!chemistry) {
        error_voltage = SOH * (vk - Ucell) / 30;
    }
    /*LFP correction factor*/
    else {
        error_voltage = SOH * (vk - Ucell) / 30;
    }
    //-------------------------------------------------------------------------
    // EKF Step 2a : Sensibility vector L which is the correction factor for the vector xhat
    // a. Compute the partial derivative of the voltage in function of SOC
    data_blockEKF.Chat[0] = 1;  //Partial derivative of URC1 in fct of SOC
    data_blockEKF.Chat[1] = 1;  //Partial derivative of URC2 in fct of SOC
    if (SOC == data_blockEKF.xhat[2] || data_blockECM.OCV_old == 0) {
        data_blockEKF.Chat[2] = 0;  //Partial derivative of OCV in fct of SOC
    } else {
        data_blockEKF.Chat[2] = (data_blockECM.OCV - data_blockECM.OCV_old) /
                                (data_blockEKF.xhat[2] - SOC);  // derivative of the OCV as a function of the SOC
    }
    data_blockEKF.Dhat = R0_SOH;
    float SigmaY;
    for (int col = 0; col < 3; col++) {
        for (int row = 0; row < 3; row++) {
            SigmaY = SigmaY + data_blockEKF.Chat[col] * data_blockEKF.SigmaX[row][col] * data_blockEKF.Chat[row];
        }
    }
    SigmaY += data_blockEKF.Dhat * data_blockEKF.SigmaV * data_blockEKF.Dhat;
    // Compute the sensibility vector L
    float L[3];
    for (int col = 0; col < 3; col++) {
        for (int row = 0; row < 3; row++) {
            L[col] = fabs(data_blockEKF.SigmaX[row][col] * data_blockEKF.Chat[col] / SigmaY) + L[col];
        }
    }
    // ------------------------------------------------------------------------
    // EKF Step 2b : Correct the state vector xhat
    for (int row = 0; row < 3; row++) {
        data_blockEKF.xhat[row] += L[row] * error_voltage;
    }
    // Boundaries conditions
    data_blockEKF.xhat[0] = fmaxf(fminf(data_blockEKF.xhat[0], 0.02f), -0.02f);
    data_blockEKF.xhat[1] = fmaxf(fminf(data_blockEKF.xhat[1], 0.02f), -0.02f);
    data_blockEKF.xhat[2] = fmaxf(fminf(data_blockEKF.xhat[2], 1.05f), 0.0);
    //-------------------------------------------------------------------------
    // EKF Step 2c : Q-bump upgrade on SigmaX
    for (int col = 0; col < 3; col++) {
        for (int row = 0; row < 3; row++) {
            data_blockEKF.SigmaX[row][col] -= L[col] * SigmaY * L[row];
        }
    }
    if (pow(error_voltage, 2) > 4 * SigmaY)  // bad voltage estimate by 2 std. devs, bump Q
    {
        data_blockEKF.SigmaX[2][2] = data_blockEKF.SigmaX[2][2] * Qbump;
    }
    /*// SVD computation
    float U[3][3], S[3], VT[3][3];
    jacobi(data_blockEKF.SigmaX, U, S, VT);
    float HH[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            float sum = 0.0;
            for (int k = 0; k < 3; k++) {
                sum += VT[i][k] * S[k] * VT[j][k];
            }
            HH[i][j] = sum;
        }
    }
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            data_blockEKF.SigmaX[i][j] =
                (data_blockEKF.SigmaX[i][j] + data_blockEKF.SigmaX[j][i] + HH[i][j] + HH[j][i]) / 4;
        }
    }*/
    //EKF ending --------------------------------------------------------------
    // SOC computation by integration of the current
    float SOC_change = delat_t * (Current_mA + data_blockECM.Current_mA_old) * (4.629629e-8) /
                       data_blockECM.SOH;                                                 //3600 * 3 * 10 * 2
    data_blockECM.SOC_int = data_blockECM.SOC_int_old - roundf(SOC_change * 1e4) * 1e-4;  // round to 0.0001
    if (data_blockECM.SOC_int < 0) {                                                      // avoid to get negative SOC
        data_blockECM.SOC_int = 0;
    }
    //---------------------------------------------------------------------
    //Check the SOH for the NMC chemistry at 25 degrees
    if (!chemistry && temperature_cell > 17.5 && temperature_cell < 35) {
        // SOC reference (min or max value)
        if (data_blockEKF.xhat[2] >= 0.98) {  // above this value, the error due to the degradation is insignificant
            data_blockECM.SOC_ref       = data_blockEKF.xhat[2];
            data_blockECM.SOC_int       = data_blockEKF.xhat[2];
            data_blockECM.SOC_ref_valid = true;
        } else if (data_blockEKF.xhat[2] <= 0.02) {
            data_blockECM.SOC_ref       = data_blockEKF.xhat[2];
            data_blockECM.SOC_int       = data_blockEKF.xhat[2];
            data_blockECM.SOC_ref_valid = true;
        }
        //---------------------------------------------------------------------
        float actual_SOC = data_blockEKF.xhat[2];
        if (data_blockECM.SOC_ref_valid && fabs(data_blockECM.SOC_ref - actual_SOC) > 0.3 && actual_SOC > 0.1 &&
            actual_SOC <= 0.9 && ik != 0) {
            /*Miss voltage due to the degradation*/
            float Uerror               = (vk - Ucell);
            float SOH_test             = 0.5;
            float U_SOH_R              = 0;
            float U_SOH_OCV            = 0;
            float SOH_first_estimation = 1;
            for (int i = 0; i < 101; i++) {
                SOH_test += i * 0.01;  // The real SOH is unknown, so it is necessary to test all the possibility
                U_SOH_R = -ik * (1 - SOH_test) *
                          (data_blockECM.R0 * 0.075 + data_blockECM.R1 * 2.5 + data_blockECM.R0 * 11);
                U_SOH_OCV = -OCV_SOH + (1.3757e-6 * powf(SOC, 4) + 1.4129e-05 * powf(SOC, 2) - 0.0357 * powf(SOC, 2) +
                                        1.7142 * SOC + (1 - SOH) * 250) /
                                           1000;
                if (Uerror > Uerror - (U_SOH_OCV - U_SOH_R)) {
                    SOH_first_estimation = SOH_test;
                    Uerror               = Uerror - (U_SOH_OCV - U_SOH_R);
                }
            }
            //-----------------------------------------------------------------
            // Comparison of the SOC progression btw current integration EKF
            float SOH_second_estimation = 1;
            if (data_blockECM.SOC_ref >= 0.95) {
                SOH_second_estimation = (data_blockECM.SOC_ref - data_blockECM.SOC_int) /
                                        (data_blockECM.SOC_ref - actual_SOC) * data_blockECM.SOH;
            } else {
                SOH_second_estimation = (data_blockECM.SOC_int / actual_SOC) * data_blockECM.SOH;
            }
            //-----------------------------------------------------------------
            // Verification of both estimation, then update the SOH value
            if (fabs(SOH_first_estimation - SOH_second_estimation) <= 0.02)  //maximum of 2% difference
            {
                data_blockECM.SOH = (SOH_first_estimation + SOH_second_estimation) * 0.5;
            }
        }
    }
    //-------------------------------------------------------------------------
    // Control of the SOC value for the LFP chemistry
    if (chemistry && !data_blockECM.SOC_validate_LFP) {
        if (roundf(ik * 1e3) * 1e3 != 0) {
            if (data_blockEKF.xhat[2] > 0.96 || (data_blockEKF.xhat[2] < 0.68 && data_blockEKF.xhat[2] > 0.65)) {
                // Compute the OCV from the measurement
                float Uocv_est = (vk + Uro + data_blockEKF.xhat[0] + data_blockEKF.xhat[1] -
                                  signbit(ik) * data_blockECM.U_hyst + data_blockECM.OCV) *
                                 0.5;
                // Select the most suitable value in the corresponding OCV look-up table
                float OCV_V   = 0;
                float new_SOC = data_blockEKF.xhat[2];
                float diff    = fabs(Uocv_est - data_blockECM.OCV);
                if (diff > 0) {
                    for (int i = 0; i < 101; i++) {
                        if (data_blockECM.temperature_set == 45) {
                            OCV_V = (float)OCVT45LFPLookupTable[i].OCV_mV / 1000;
                        } else if (data_blockECM.temperature_set == 25) {
                            OCV_V = (float)OCVT25LFPLookupTable[i].OCV_mV / 1000;
                        } else {
                            OCV_V = (float)OCVT0LFPLookupTable[i].OCV_mV / 1000;
                        }
                        if (diff > fabs(Uocv_est - OCV_V)) {
                            diff    = fabs(Uocv_est - OCV_V);
                            new_SOC = (float)(i / 100);
                        }
                        if (fabs(new_SOC - data_blockEKF.xhat[2]) < 0.02) {
                            data_blockECM.SOC_validate_LFP = true;
                        }
                        data_blockEKF.xhat[2] = (data_blockEKF.xhat[2] + new_SOC) * 0.5;
                    }
                } else {
                    data_blockECM.SOC_validate_LFP = true;
                }
            }
        }
    }

    //-------------------------------------------------------------------------
    //Update database
    data_blockECM.OCV_old        = data_blockECM.OCV;
    data_blockECM.Uest           = Ucell;
    data_blockECM.Current_mA_old = Current_mA;
    data_blockECM.SOC_EKF_old    = SOC;
    data_blockECM.SOC_int_old    = data_blockECM.SOC_int;
    data_blockECM.SOC_EKF        = data_blockEKF.xhat[2];
    DATA_WRITE_DATA(&data_blockEKF);
    DATA_WRITE_DATA(&data_blockECM);
}

//-----------------------------------------------------------------------------
// In function of the SOC, Current, and Temperature, the corresponding parameters for the ECM and the OCV are chosen
extern void Parameters_selection(bool chemistry, int16_t temperature_cell, float SOC) {
    DATA_BLOCK_ECM_s data_blockECM = {.header.uniqueId = DATA_BLOCK_ID_ECM};
    DATA_READ_DATA(&data_blockECM);
    int SOC_percent = (int)(SOC * 100);
    if (SOC_percent > 100) {
        SOC_percent = 100;
    }

    if (!chemistry) /*NMC chemistry*/
    {
        // 45 degrees
        if (temperature_cell > 37) {
            data_blockECM.temperature_set = 45;
            data_blockECM.OCV             = (float)OCVT45NMCLookupTable[(SOC_percent)].OCV_mV / 1000;
            data_blockECM.R0              = (float)ECMT45NMCLookupTable[(SOC_percent)].R0 / 1000;
            data_blockECM.R1              = (float)ECMT45NMCLookupTable[(SOC_percent)].R1 / 1000;
            data_blockECM.R2              = (float)ECMT45NMCLookupTable[(SOC_percent)].R2 / 1000;
            data_blockECM.Tau1            = (float)ECMT45NMCLookupTable[(SOC_percent)].Tau1 / 1000;
            data_blockECM.Tau2            = (float)ECMT45NMCLookupTable[(SOC_percent)].Tau2 / 1000;
            float a                       = HystNMCLookupTable[3].a;
            float b                       = SOC - HystNMCLookupTable[3].b;
            float c                       = SOC - HystNMCLookupTable[3].c;
            float d                       = SOC - HystNMCLookupTable[3].d;
            float e                       = SOC - HystNMCLookupTable[3].e;
            float f                       = SOC - HystNMCLookupTable[3].f;
            float g                       = SOC - HystNMCLookupTable[3].g;
            data_blockECM.U_hyst = a + powf(b, 2) - powf(c, 3) + powf(d, 4) - powf(e, 5) + powf(f, 6) - powf(g, 7);
        }
        // 25 degrees
        else if (temperature_cell > 17 && temperature_cell < 38) {
            data_blockECM.temperature_set = 25;
            data_blockECM.OCV             = (float)OCVT25NMCLookupTable[(SOC_percent)].OCV_mV / 1000;
            data_blockECM.R0              = (float)ECMT25NMCLookupTable[(SOC_percent)].R0 / 1000;
            data_blockECM.R1              = (float)ECMT25NMCLookupTable[(SOC_percent)].R1 / 1000;
            data_blockECM.R2              = (float)ECMT25NMCLookupTable[(SOC_percent)].R2 / 1000;
            data_blockECM.Tau1            = (float)ECMT25NMCLookupTable[(SOC_percent)].Tau1 / 1000;
            data_blockECM.Tau2            = (float)ECMT25NMCLookupTable[(SOC_percent)].Tau2 / 1000;
            float a                       = HystNMCLookupTable[2].a;
            float b                       = SOC - HystNMCLookupTable[2].b;
            float c                       = SOC - HystNMCLookupTable[2].c;
            float d                       = SOC - HystNMCLookupTable[2].d;
            float e                       = SOC - HystNMCLookupTable[2].e;
            float f                       = SOC - HystNMCLookupTable[2].f;
            float g                       = SOC - HystNMCLookupTable[2].g;
            data_blockECM.U_hyst = a + powf(b, 2) - powf(c, 3) + powf(d, 4) - powf(e, 5) + powf(f, 6) - powf(g, 7);
        }
        // 10 degrees
        else if (temperature_cell > 4 && temperature_cell < 18) {
            data_blockECM.temperature_set = 10;
            data_blockECM.OCV             = (float)OCVT10NMCLookupTable[(SOC_percent)].OCV_mV / 1000;
            data_blockECM.R0              = (float)ECMT10NMCLookupTable[(SOC_percent)].R0 / 1000;
            data_blockECM.R1              = (float)ECMT10NMCLookupTable[(SOC_percent)].R1 / 1000;
            data_blockECM.R2              = (float)ECMT10NMCLookupTable[(SOC_percent)].R2 / 1000;
            data_blockECM.Tau1            = (float)ECMT10NMCLookupTable[(SOC_percent)].Tau1 / 1000;
            data_blockECM.Tau2            = (float)ECMT10NMCLookupTable[(SOC_percent)].Tau2 / 1000;
            float a                       = HystNMCLookupTable[1].a;
            float b                       = SOC - HystNMCLookupTable[1].b;
            float c                       = SOC - HystNMCLookupTable[1].c;
            float d                       = SOC - HystNMCLookupTable[1].d;
            float e                       = SOC - HystNMCLookupTable[1].e;
            float f                       = SOC - HystNMCLookupTable[1].f;
            float g                       = SOC - HystNMCLookupTable[1].g;
            data_blockECM.U_hyst = a + powf(b, 2) - powf(c, 3) + powf(d, 4) - powf(e, 5) + powf(f, 6) - powf(g, 7);
        }
        // 0 degrees
        else {
            data_blockECM.temperature_set = 0;
            data_blockECM.OCV             = (float)OCVT0NMCLookupTable[(SOC_percent)].OCV_mV / 1000;
            data_blockECM.R0              = (float)ECMT0NMCLookupTable[(SOC_percent)].R0 / 1000;
            data_blockECM.R1              = (float)ECMT0NMCLookupTable[(SOC_percent)].R1 / 1000;
            data_blockECM.R2              = (float)ECMT0NMCLookupTable[(SOC_percent)].R2 / 1000;
            data_blockECM.Tau1            = (float)ECMT0NMCLookupTable[(SOC_percent)].Tau1 / 1000;
            data_blockECM.Tau2            = (float)ECMT0NMCLookupTable[(SOC_percent)].Tau2 / 1000;
            float a                       = HystNMCLookupTable[0].a;
            float b                       = SOC - HystNMCLookupTable[0].b;
            float c                       = SOC - HystNMCLookupTable[0].c;
            float d                       = SOC - HystNMCLookupTable[0].d;
            float e                       = SOC - HystNMCLookupTable[0].e;
            float f                       = SOC - HystNMCLookupTable[0].f;
            float g                       = SOC - HystNMCLookupTable[0].g;
            data_blockECM.U_hyst = a + powf(b, 2) - powf(c, 3) + powf(d, 4) - powf(e, 5) + powf(f, 6) - powf(g, 7);
        }
    } else { /*LFP chemistry*/
        // 45 degrees
        if (temperature_cell > 37) {
            data_blockECM.OCV    = (float)OCVT45LFPLookupTable[(SOC_percent)].OCV_mV / 1000;
            data_blockECM.R0     = (float)ECMT45LFPLookupTable[(SOC_percent)].R0 / 1000;
            data_blockECM.R1     = (float)ECMT45LFPLookupTable[(SOC_percent)].R1 / 1000;
            data_blockECM.R2     = (float)ECMT45LFPLookupTable[(SOC_percent)].R2 / 1000;
            data_blockECM.Tau1   = (float)ECMT45LFPLookupTable[(SOC_percent)].Tau1 / 1000;
            data_blockECM.Tau2   = (float)ECMT45LFPLookupTable[(SOC_percent)].Tau2 / 1000;
            float a              = HystLFPLookupTable[2].a;
            float b              = SOC - HystLFPLookupTable[2].b;
            float c              = SOC - HystLFPLookupTable[2].c;
            float d              = SOC - HystLFPLookupTable[2].d;
            float e              = SOC - HystLFPLookupTable[2].e;
            float f              = SOC - HystLFPLookupTable[2].f;
            float g              = SOC - HystLFPLookupTable[2].g;
            data_blockECM.U_hyst = a + powf(b, 2) - powf(c, 3) + powf(d, 4) - powf(e, 5) + powf(f, 6) - powf(g, 7);
        }
        // 25 degrees
        else if (temperature_cell > 12 && temperature_cell < 38) {
            data_blockECM.OCV    = (float)OCVT25LFPLookupTable[(SOC_percent)].OCV_mV / 1000;
            data_blockECM.R0     = (float)ECMT25LFPLookupTable[(SOC_percent)].R0 / 1000;
            data_blockECM.R1     = (float)ECMT25LFPLookupTable[(SOC_percent)].R1 / 1000;
            data_blockECM.R2     = (float)ECMT25LFPLookupTable[(SOC_percent)].R2 / 1000;
            data_blockECM.Tau1   = (float)ECMT25LFPLookupTable[(SOC_percent)].Tau1 / 1000;
            data_blockECM.Tau2   = (float)ECMT25LFPLookupTable[(SOC_percent)].Tau2 / 1000;
            float a              = HystLFPLookupTable[1].a;
            float b              = SOC - HystLFPLookupTable[1].b;
            float c              = SOC - HystLFPLookupTable[1].c;
            float d              = SOC - HystLFPLookupTable[1].d;
            float e              = SOC - HystLFPLookupTable[1].e;
            float f              = SOC - HystLFPLookupTable[1].f;
            float g              = SOC - HystLFPLookupTable[1].g;
            data_blockECM.U_hyst = a + powf(b, 2) - powf(c, 3) + powf(d, 4) - powf(e, 5) + powf(f, 6) - powf(g, 7);
        }
        // 0 degrees
        else {
            data_blockECM.OCV    = (float)OCVT0LFPLookupTable[(SOC_percent)].OCV_mV / 1000;
            data_blockECM.R0     = (float)ECMT0LFPLookupTable[(SOC_percent)].R0 / 1000;
            data_blockECM.R1     = (float)ECMT0LFPLookupTable[(SOC_percent)].R1 / 1000;
            data_blockECM.R2     = (float)ECMT0LFPLookupTable[(SOC_percent)].R2 / 1000;
            data_blockECM.Tau1   = (float)ECMT0LFPLookupTable[(SOC_percent)].Tau1 / 1000;
            data_blockECM.Tau2   = (float)ECMT0LFPLookupTable[(SOC_percent)].Tau2 / 1000;
            float a              = HystLFPLookupTable[0].a;
            float b              = SOC - HystLFPLookupTable[0].b;
            float c              = SOC - HystLFPLookupTable[0].c;
            float d              = SOC - HystLFPLookupTable[0].d;
            float e              = SOC - HystLFPLookupTable[0].e;
            float f              = SOC - HystLFPLookupTable[0].f;
            float g              = SOC - HystLFPLookupTable[0].g;
            data_blockECM.U_hyst = a + powf(b, 2) - powf(c, 3) + powf(d, 4) - powf(e, 5) + powf(f, 6) - powf(g, 7);
        }
    }
    DATA_WRITE_DATA(&data_blockECM);
}

// SVD computation
void jacobi(float A[M][N], float U[M][M], float S[N], float VT[N][N]) {
    // Initialize U and VT as identity matrices
    for (int i = 0; i < M; i++) {
        for (int j = 0; j < M; j++) {
            U[i][j] = (i == j) ? 1.0 : 0.0;
        }
    }
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            VT[i][j] = (i == j) ? 1.0 : 0.0;
        }
    }

    // Initialize S as the diagonal of A
    for (int i = 0; i < N; i++) {
        S[i] = A[i][i];
    }

    // Perform Jacobi rotations until convergence
    int max_iter  = 100;
    float tol     = 1e-6;
    int converged = 0;
    for (int k = 0; k < max_iter && !converged; k++) {
        converged = 1;
        for (int p = 0; p < N - 1; p++) {
            for (int q = p + 1; q < N; q++) {
                float apq   = A[p][q];
                float app   = A[p][p];
                float aqq   = A[q][q];
                float theta = 0.5 * atan2(2 * apq, aqq - app);
                float c     = cos(theta);
                float s     = sin(theta);

                // Update A, U, and VT
                for (int i = 0; i < N; i++) {
                    float api = A[p][i];
                    float aqi = A[q][i];
                    A[p][i]   = c * api + s * aqi;
                    A[q][i]   = -s * api + c * aqi;
                }
                for (int i = 0; i < M; i++) {
                    float upi = U[i][p];
                    float uqi = U[i][q];
                    U[i][p]   = c * upi + s * uqi;
                    U[i][q]   = -s * upi + c * uqi;
                }
                for (int i = 0; i < N; i++) {
                    float vti = VT[i][p];
                    float vtj = VT[i][q];
                    VT[i][p]  = c * vti + s * vtj;
                    VT[i][q]  = -s * vti + c * vtj;
                }

                // Check convergence
                float apq_new = A[p][q];
                if (fabs(apq_new - apq) > tol) {
                    converged = 0;
                }
            }
        }
    }
}
/*========== Externalized Static Function Implementations (Unit Test) =======*/
#ifdef UNITY_UNIT_TEST
extern void TEST_ALGO_ResetInitializationRequest() {
    algo_initializationRequested = false;
}
#endif /* UNITY_UNIT_TEST */
