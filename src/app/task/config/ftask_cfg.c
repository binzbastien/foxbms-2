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
 * @file    ftask_cfg.c
 * @author  foxBMS Team
 * @date    2019-08-26 (date of creation)
 * @updated 2022-10-27 (date of last update)
 * @version v1.4.1
 * @ingroup TASK_CONFIGURATION
 * @prefix  FTSK
 *
 * @brief   Task configuration
 * @details
 */

/*========== Includes =======================================================*/
#include "ftask_cfg.h"

#include "battery_cell_cfg.h"

#include "HL_gio.h"
#include "HL_het.h"

#include "adc.h"
#include "algorithm.h"
#include "bal.h"
#include "bms.h"
#include "can.h"
#include "contactor.h"
#include "database.h"
#include "diag.h"
#include "dma.h"
#include "fram.h"
#include "htsensor.h"
#include "i2c.h"
#include "imd.h"
#include "interlock.h"
#include "led.h"
#include "meas.h"
#include "pex.h"
#include "redundancy.h"
#include "sbc.h"
#include "sof_trapezoid.h"
#include "spi.h"
#include "sps.h"
#include "state_estimation.h"
#include "sys.h"
#include "sys_mon.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*========== Macros and Definitions =========================================*/
#define _OPEN_SYS_ITOA_EXT

/** counter value for 50ms in 10ms task */
#define TASK_10MS_COUNTER_FOR_50MS (5u)

/** counter value for 1s in 100ms task */
#define TASK_100MS_COUNTER_FOR_1S (10u)

/*========== Static Constant and Variable Definitions =======================*/

/*========== Extern Constant and Variable Definitions =======================*/

/**
 * @brief   Definition of the engine task
 * @details Task is not delayed after the scheduler starts. This task  must
 *          have the highest priority.
 * @warning Do not change the configuration of this task. This will very
 *          likely break the system.
 */
OS_TASK_DEFINITION_s ftsk_taskDefinitionEngine = {
    FTSK_TASK_ENGINE_PRIORITY,
    FTSK_TASK_ENGINE_PHASE,
    FTSK_TASK_ENGINE_CYCLE_TIME,
    FTSK_TASK_ENGINE_STACK_SIZE_IN_BYTES,
    FTSK_TASK_ENGINE_PV_PARAMETERS};
OS_TASK_DEFINITION_s ftsk_taskDefinitionCyclic1ms = {
    FTSK_TASK_CYCLIC_1MS_PRIORITY,
    FTSK_TASK_CYCLIC_1MS_PHASE,
    FTSK_TASK_CYCLIC_1MS_CYCLE_TIME,
    FTSK_TASK_CYCLIC_1MS_STACK_SIZE_IN_BYTES,
    FTSK_TASK_CYCLIC_1MS_PV_PARAMETERS};
OS_TASK_DEFINITION_s ftsk_taskDefinitionCyclic10ms = {
    FTSK_TASK_CYCLIC_10MS_PRIORITY,
    FTSK_TASK_CYCLIC_10MS_PHASE,
    FTSK_TASK_CYCLIC_10MS_CYCLE_TIME,
    FTSK_TASK_CYCLIC_10MS_STACK_SIZE_IN_BYTES,
    FTSK_TASK_CYCLIC_10MS_PV_PARAMETERS};
OS_TASK_DEFINITION_s ftsk_taskDefinitionCyclic100ms = {
    FTSK_TASK_CYCLIC_100MS_PRIORITY,
    FTSK_TASK_CYCLIC_100MS_PHASE,
    FTSK_TASK_CYCLIC_100MS_CYCLE_TIME,
    FTSK_TASK_CYCLIC_100MS_STACK_SIZE_IN_BYTES,
    FTSK_TASK_CYCLIC_100MS_PV_PARAMETERS};
OS_TASK_DEFINITION_s ftsk_taskDefinitionCyclicAlgorithm100ms = {
    FTSK_TASK_CYCLIC_ALGORITHM_100MS_PRIORITY,
    FTSK_TASK_CYCLIC_ALGORITHM_100MS_PHASE,
    FTSK_TASK_CYCLIC_ALGORITHM_100MS_CYCLE_TIME,
    FTSK_TASK_CYCLIC_ALGORITHM_100MS_STACK_SIZE_IN_BYTES,
    FTSK_TASK_CYCLIC_ALGORITHM_100MS_PV_PARAMETERS};
OS_TASK_DEFINITION_s ftsk_taskDefinitionAfe = {
    FTSK_TASK_AFE_PRIORITY,
    FTSK_TASK_AFE_PHASE,
    FTSK_TASK_AFE_CYCLE_TIME,
    FTSK_TASK_AFE_STACK_SIZE_IN_BYTES,
    FTSK_TASK_AFE_PV_PARAMETERS};

/*========== Static Function Prototypes =====================================*/

/*========== Static Constant and Variable Definitions =======================*/
// Sensors-----------------------------------------------------------------
DATA_BLOCK_CELL_VOLTAGE_s table_voltage_slave1         = {.header.uniqueId = DATA_BLOCK_ID_CELL_VOLTAGE};
DATA_BLOCK_CELL_TEMPERATURE_s table_temperature_slave1 = {.header.uniqueId = DATA_BLOCK_ID_CELL_TEMPERATURE};
//Database-----------------------------------------------------------------
DATA_BLOCK_DATABASE_s data_blockDatabase                   = {.header.uniqueId = DATA_BLOCK_ID_DATABASE};
DATA_BLOCK_ECM_s data_blockECM                             = {.header.uniqueId = DATA_BLOCK_ID_ECM};
DATA_BLOCK_RD_BMS_OP_TIME_s data_blockrdBmsOpTime          = {.header.uniqueId = DATA_BLOCK_ID_RD_BMS_OP_TIME};
DATA_BLOCK_RD_BMS_CONSUMPTION_s data_blockrdBmsConsumption = {.header.uniqueId = DATA_BLOCK_ID_RD_BMS_CONSUMPTION};
DATA_BLOCK_RD_BMS_EOC_s data_blockrdBmsEoc                 = {.header.uniqueId = DATA_BLOCK_ID_RD_BMS_EOC};
DATA_BLOCK_RD_BMS_EOD_s data_blockrdBmsEod                 = {.header.uniqueId = DATA_BLOCK_ID_RD_BMS_EOD};
DATA_BLOCK_RD_BMS_DOD_s data_blockrdBmsDod                 = {.header.uniqueId = DATA_BLOCK_ID_RD_BMS_DOD};
DATA_BLOCK_RD_BMS_DOC_s data_blockrdBmsDoc                 = {.header.uniqueId = DATA_BLOCK_ID_RD_BMS_DOC};
DATA_BLOCK_RD_BMS_CUMTIME_DISCHARGE_s data_blockrdBmsCumtimeDischarge = {
    .header.uniqueId = DATA_BLOCK_ID_RD_BMS_CUMTIME_DISCHARGE};
DATA_BLOCK_RD_BMS_CUMTIME_CHARGE_s data_blockrdBmsCumtimeCharge = {
    .header.uniqueId = DATA_BLOCK_ID_RD_BMS_CUMTIME_CHARGE};
DATA_BLOCK_RD_BMS_CUMTIME_REGEN_s data_blockrdBmsCumtimeRegen = {.header.uniqueId = DATA_BLOCK_ID_RD_BMS_CUMTIME_REGEN};
DATA_BLOCK_RD_BMS_CUMTIME_OPTEMP_s data_blockrdBmsCumtimeOptemp = {
    .header.uniqueId = DATA_BLOCK_ID_RD_BMS_CUMTIME_OPTEMP};
DATA_BLOCK_RD_BMS_CUMTIME_CHARGE_TEMP_s data_blockrdBmsCumtimeCharge_temp = {
    .header.uniqueId = DATA_BLOCK_ID_RD_BMS_CUMTIME_CHARGE_TEMP};
DATA_BLOCK_RD_BMS_CUMTIME_DISCHARGE_TEMP_s data_blockrdBmsCumtimeDischarge_temp = {
    .header.uniqueId = DATA_BLOCK_ID_RD_BMS_CUMTIME_DISCHARGE_TEMP};
DATA_BLOCK_RD_BMS_CUMTIME_REGEN_TEMP_s data_blockrdBmsCumtimeRegen_temp = {
    .header.uniqueId = DATA_BLOCK_ID_RD_BMS_CUMTIME_REGEN_TEMP};
DATA_BLOCK_RD_BMS_CUMTIME_SOC_s data_blockrdBmsCumtimeSoc = {.header.uniqueId = DATA_BLOCK_ID_RD_BMS_CUMTIME_SOC};
/*========== Static Function Implementations ================================*/

/*========== Extern Function Implementations ================================*/
extern void FTSK_InitializeUserCodeEngine(void) {
    /* Warning: Do not change the content of this function */
    /* See function definition doxygen comment for details */
    STD_RETURN_TYPE_e retval = DATA_Initialize();

    if (retval == E_NOT_OK) {
        /* Fatal error! */
        FAS_ASSERT(FAS_TRAP);
    }

    /* Suspend AFE task if unused, otherwise it will preempt all lower priority tasks */
#if (FOXBMS_AFE_DRIVER_TYPE_FSM == 1)
    vTaskSuspend(ftsk_taskHandleAfe);
#endif

    /* Init FRAM */
    FRAM_Initialize();

    retval = SYSM_Init();

    if (retval == E_NOT_OK) {
        /* Fatal error! */
        FAS_ASSERT(FAS_TRAP);
    }

    /* Warning: Do not change the content of this function */
    /* See function definition doxygen comment for details */
}

extern void FTSK_RunUserCodeEngine(void) {
    /* Warning: Do not change the content of this function */
    /* See function definition doxygen comment for details */
    DATA_Task();               /* Call database manager */
    SYSM_CheckNotifications(); /* Check notifications from tasks */
    /* Warning: Do not change the content of this function */
    /* See function definition doxygen comment for details */
}

extern void FTSK_InitializeUserCodePreCyclicTasks(void) {
    /* user code */
    SYS_RETURN_TYPE_e sys_retVal = SYS_ILLEGAL_REQUEST;

    /*  Init Sys */
    sys_retVal = SYS_SetStateRequest(SYS_STATE_INITIALIZATION_REQUEST);

    /* Init port expander */
    PEX_Initialize();

    /* Set 3rd PE pin to activate temperature/humidity sensor */
    PEX_SetPinDirectionOutput(PEX_PORT_EXPANDER3, PEX_PIN00);
    PEX_SetPin(PEX_PORT_EXPANDER3, PEX_PIN00);

    CONT_Initialize();
    SPS_Initialize();
    (void)MEAS_Initialize(); /* cast to void as the return value is unused */

    /* Initialize redundancy module */
    (void)MRC_Initialize();

    /* This function operates under the assumption that it is called when
     * the operating system is not yet running.
     * In this state the return value of #SYS_SetStateRequest should
     * always be #SYS_OK. Therefore we trap otherwise.
     */
    FAS_ASSERT(sys_retVal == SYS_OK);

    /* System started correctly -> Start toggling of debug LED */
    LED_SetToggleTime(LED_NORMAL_OPERATION_ON_OFF_TIME_ms);
}

extern void FTSK_RunUserCodeCyclic1ms(void) {
    /* Increment of operating system timer */
    /* This must not be changed, add user code only below */
    OS_IncrementTimer();
    DIAG_UpdateFlags();
    /* user code */
#if (FOXBMS_AFE_DRIVER_TYPE_FSM == 1)
    MEAS_Control();
#endif
    CAN_ReadRxBuffer();
}

extern void FTSK_RunUserCodeCyclic10ms(void) {
    static uint8_t ftsk_cyclic10msCounter = 0;
    /* user code */
    SYSM_UpdateFramData();
    SYS_Trigger(&sys_state);
    ILCK_Trigger();
    ADC_Control();
    SPS_Ctrl();
    CAN_MainFunction();
    SOF_Calculation();
    ALGO_MonitorExecutionTime();
    SBC_Trigger(&sbc_stateMcuSupervisor);
    PEX_Trigger();
    HTSEN_Trigger();
    if (ftsk_cyclic10msCounter == TASK_10MS_COUNTER_FOR_50MS) {
        MRC_ValidateAfeMeasurement();
        MRC_ValidatePackMeasurement();
        ftsk_cyclic10msCounter = 0;
    }
    /* Call BMS_Trigger function at the end of the 10ms task to allow previously
     * called modules in this task to update respectively evaluate their new.
     * This minimizes the delay between data evaluation and the reaction from
     * the BMS module */
    BMS_Trigger();
    ftsk_cyclic10msCounter++;
}

extern void FTSK_RunUserCodeCyclic100ms(void) {
    /* user code */
    static uint8_t ftsk_cyclic100msCounter = 0;

    /** Perform SOC and SOE calculations only every 1s. Not suited if analog
     *  integration of current sensor is NOT used. Manual integration of current
     *  requires a higher frequency.
     */
    if (ftsk_cyclic100msCounter == TASK_100MS_COUNTER_FOR_1S) {
        SE_RunStateEstimations();
        ftsk_cyclic100msCounter = 0;
    }

    BAL_Trigger();
    IMD_Trigger();
    LED_Trigger();

    ftsk_cyclic100msCounter++;
}
//-----------------------------------------------------------------------------
extern void FTSK_RunUserCodeCyclicAlgorithm100ms(void) {
    static uint16_t ftsk_cyclicAlgorithm100msCounter = 650;
    static int select_file                           = 0; /*!< Select the file where to save the database*/
    FILE *fp;
    DATA_READ_DATA(&table_voltage_slave1);      // Read Voltage
    DATA_READ_DATA(&table_temperature_slave1);  // Read temperature
    /* user code */
    // Initialization (50ms to read all the inputs)
    if (ftsk_cyclicAlgorithm100msCounter >= 698) {
        DATA_read_BIN_file(select_file);  // Recovery the database
        select_file++;
        if (select_file == 3) {
            select_file = 0;
        }
    }
    //---------------------------------------------------------------------
    // Read the latest value of SOC
    if (ftsk_cyclicAlgorithm100msCounter == 700) {
        fp = fopen("SOC_last_value.bin", "rb");
        if (fp == NULL) {
            data_blockECM.SOC_int_old = 50;
        } else {
            fread(&data_blockECM.SOC_EKF_old, sizeof(float), 1, fp);
            data_blockECM.SOC_EKF     = data_blockECM.SOC_EKF_old;
            data_blockECM.SOC_int_old = data_blockECM.SOC_EKF_old;
            data_blockECM.SOH         = 1;
        }
        fclose(fp);
        DATA_WRITE_DATA(&data_blockECM);  // Read the new SOC
        ftsk_cyclicAlgorithm100msCounter = 0;
    }
    // Update of the SOC state and the database each second
    if (ftsk_cyclicAlgorithm100msCounter % 10 == 0 && ftsk_cyclicAlgorithm100msCounter <= 600) {
        DATA_READ_DATA(&data_blockDatabase);
        DATA_READ_DATA(&data_blockECM);
        //---------------------------------------------------------------------
        // Read values
        DATA_READ_DATA(&table_voltage_slave1);                              // Read Voltage
        DATA_READ_DATA(&table_temperature_slave1);                          // Read temperature
        int16_t voltage_cell1 = table_voltage_slave1.cellVoltage_mV[0][3];  // Voltage on the first cell
        int8_t temperature_cell1 =
            (uint8_t)(table_temperature_slave1.cellTemperature_ddegC[0][0] * 0.1 + 10);  // Temperature of the cell in degree
        int16_t Current_volt = table_voltage_slave1.cellVoltage_mV[0][0];                // Read Current
        int16_t Current_mA   = (Current_volt - 2500) * 10 - 140;  // Conversion mV to mA minus offset
        //-------------------------------------------------------------------------
        // Parameters selection
        float enen = fabs(data_blockECM.last_SOC_ECM_check - data_blockECM.SOC_EKF_old);
        if (fabs(data_blockECM.last_SOC_ECM_check - data_blockECM.SOC_EKF_old) >= 0.01) {
            Parameters_selection(data_blockECM.chemistry, temperature_cell1, data_blockECM.SOC_EKF);
            data_blockECM.last_SOC_ECM_check = data_blockECM.SOC_EKF_old;
        }
        //---------------------------------------------------------------------
        //EKF
        DATA_READ_DATA(&data_blockECM);  // Update the data for the function
        ALGO_bfh_SOC_NMC_LFP(data_blockECM.chemistry, temperature_cell1, Current_mA, voltage_cell1);
        DATA_READ_DATA(&data_blockECM);  // Read the new SOC
        //Send the measurements and the estimation to analyze the efficiency of the algorithm
        uint8_t Data_process[8] = {
            (uint8_t)(Current_volt >> 8),
            (uint8_t)(Current_volt),
            (uint8_t)((uint16_t)(data_blockECM.Uest * 1000) >> 8),
            (uint8_t)(data_blockECM.Uest * 1000.0f),
            (uint8_t)((uint16_t)(data_blockECM.SOC_EKF * 10000) >> 8),
            (uint8_t)(data_blockECM.SOC_EKF * 10000),
            (uint8_t)(data_blockECM.SOH * 100),
            (uint8_t)(temperature_cell1)};
        CAN_DataSend(CAN_NODE_1, 0x0ff, &Data_process[0]);
        //---------------------------------------------------------------------
        // Variable which indicate where the actual state must be save in each array of the database
        uint8_t i_I = (int)(abs(Current_mA) / 6000 * 5);  //Current index
        uint8_t i_T = (int)(temperature_cell1 / 45 * 5);  //Temperature index
        if (temperature_cell1 < 0) {                      // the lookup table has a minium of 0 degree
            i_T = 0;
        }
        uint8_t i_SOC = (int)(data_blockECM.SOC_EKF / 0.125);  //SOC index
        //---------------------------------------------------------------------
        // Update the DATA BASE
        //
        //  1. Read the DATA BASE
        DATA_READ_DATA(&data_blockrdBmsOpTime);
        DATA_READ_DATA(&data_blockrdBmsConsumption);
        DATA_READ_DATA(&data_blockrdBmsEoc);
        DATA_READ_DATA(&data_blockrdBmsEod);
        DATA_READ_DATA(&data_blockrdBmsDod);
        DATA_READ_DATA(&data_blockrdBmsDoc);
        DATA_READ_DATA(&data_blockrdBmsCumtimeDischarge);
        DATA_READ_DATA(&data_blockrdBmsCumtimeCharge);
        DATA_READ_DATA(&data_blockrdBmsCumtimeRegen);
        DATA_READ_DATA(&data_blockrdBmsCumtimeOptemp);
        DATA_READ_DATA(&data_blockrdBmsCumtimeCharge_temp);
        DATA_READ_DATA(&data_blockrdBmsCumtimeDischarge_temp);
        DATA_READ_DATA(&data_blockrdBmsCumtimeRegen_temp);
        DATA_READ_DATA(&data_blockrdBmsCumtimeSoc);
        //---------------------------------------------------------------------
        // 2. Update it
        data_blockrdBmsCumtimeSoc.SOC[i_SOC]++;
        data_blockrdBmsCumtimeOptemp.OPT[i_T]++;

        if ((Current_mA < 0) & (data_blockDatabase.RegenerativeTime > 60))
        // Charge phase -------------------------------------------------------
        {
            data_blockrdBmsCumtimeCharge.CCC[i_I]++;
            data_blockrdBmsCumtimeCharge_temp.CPT[i_T]++;
            data_blockrdBmsOpTime.HR[3]++;
            data_blockrdBmsConsumption.CONSUMPTION[0] += Current_mA * 0.001 * Current_volt * 0.001 / 3600;
            data_blockrdBmsConsumption.CONSUMPTION[3] += Current_mA * 0.001 / 3600;
            if (data_blockDatabase.DischargePhase) {
                data_blockrdBmsEod.EOD[i_SOC]++;
                data_blockrdBmsDod.DOD[8 - (int)((data_blockDatabase.Start_DOD - data_blockECM.SOC_EKF) * 8)]++;
            }
            data_blockDatabase.ChargePhase    = true;
            data_blockDatabase.DischargePhase = false;
            data_blockDatabase.SleepTime      = 0;

        } else if ((Current_mA < 0) & (data_blockDatabase.RegenerativeTime <= 60))
        // Regenerative phase or charge phase ---------------------------------
        {
            data_blockDatabase.RegenerativeTable[i_I][0]++;  // Save current
            data_blockDatabase.RegenerativeTable[i_T][1]++;  // Save temperature
            data_blockDatabase.RegenerativeTime++;
            data_blockDatabase.Ahregen += Current_mA * 0.001 / 3600;
            data_blockDatabase.Whregen += Current_mA * 0.001 * voltage_cell1 * 0.001 / 3600;
            if (data_blockDatabase.DischargePhase) {
                data_blockrdBmsEod.EOD[i_SOC]++;
                data_blockDatabase.Start_DOC = data_blockECM.SOC_EKF;
                data_blockrdBmsDod.DOD[8 - (int)((data_blockDatabase.Start_DOD - data_blockECM.SOC_EKF) * 8)]++;
            }
            data_blockDatabase.DischargePhase = false;
            data_blockDatabase.SleepTime      = 0;

        } else if (Current_mA > 0) {
            if (data_blockDatabase.RegenerativeTime > 0 && data_blockDatabase.RegenerativeTime <= 60) {
                //Regenerative phase ------------------------------------------
                for (size_t i = 0; i < 5; i++) {
                    data_blockDatabase.RegenCurrent[i] += data_blockDatabase.RegenerativeTable[i][0];
                    data_blockDatabase.TemperatureRegenCurrent[i] += data_blockDatabase.RegenerativeTable[i][1];
                    data_blockrdBmsCumtimeRegen.REG[i_I]++;
                    data_blockrdBmsCumtimeRegen_temp.REGT[i_T]++;
                    data_blockrdBmsOpTime.HR[5]++;
                }
                data_blockrdBmsConsumption.CONSUMPTION[2] += data_blockDatabase.Ahregen;
                data_blockrdBmsConsumption.CONSUMPTION[5] += data_blockDatabase.Whregen;
                data_blockDatabase.RegenerativeTime = 0;
            }
            // Discharge phase ------------------------------------------------
            data_blockrdBmsCumtimeDischarge.DCC[i_I]++;
            data_blockrdBmsCumtimeDischarge_temp.DPT[i_T]++;
            data_blockrdBmsOpTime.HR[4]++;
            data_blockrdBmsConsumption.CONSUMPTION[1] += Current_mA * 0.001 * Current_volt * 0.001 / 3600;
            data_blockrdBmsConsumption.CONSUMPTION[4] += Current_mA * 0.001 / 3600;
            if (data_blockDatabase.ChargePhase) {
                data_blockrdBmsEoc.EOC[i_SOC]++;
                data_blockDatabase.Start_DOD = data_blockECM.SOC_EKF;
                data_blockrdBmsDoc.DOC[8 - (int)((data_blockDatabase.Start_DOC - data_blockECM.SOC_EKF) * 8)]++;
            }
            data_blockDatabase.ChargePhase    = false;
            data_blockDatabase.DischargePhase = true;
            data_blockDatabase.SleepTime      = 0;

        } else {  //Current_mA == 0
            data_blockDatabase.SleepTime++;
            if (data_blockDatabase.SleepTime <= 3600 * 3) {  //idle
                data_blockrdBmsOpTime.HR[2]++;
            } else if (data_blockDatabase.SleepTime > 3600 * 3) {  //sleep
                data_blockrdBmsOpTime.HR[1]++;
                data_blockrdBmsOpTime.HR[2] = data_blockrdBmsOpTime.HR[2] - 3600 * 3;
            } else if (data_blockDatabase.SleepTime > 3600 * 24) {
                data_blockrdBmsOpTime.HR[0]++;
                data_blockrdBmsOpTime.HR[1] = data_blockrdBmsOpTime.HR[1] - 3600 * 24;
            }
            //to avoid error of the builder that it's never used
            data_blockrdBmsConsumption.CONSUMPTION[1] = data_blockrdBmsConsumption.CONSUMPTION[1];
        }
        //-------------------------------------------------------------------------
        // 3. Save it locally
        DATA_WRITE_DATA(&data_blockrdBmsOpTime, &data_blockrdBmsConsumption, &data_blockrdBmsEoc, &data_blockrdBmsEod);
        DATA_WRITE_DATA(
            &data_blockrdBmsDod, &data_blockrdBmsDoc, &data_blockrdBmsCumtimeDischarge, &data_blockrdBmsCumtimeCharge);
        DATA_WRITE_DATA(
            &data_blockrdBmsCumtimeRegen, &data_blockrdBmsCumtimeOptemp, &data_blockrdBmsCumtimeDischarge_temp);
        DATA_WRITE_DATA(
            &data_blockrdBmsCumtimeRegen_temp, &data_blockrdBmsCumtimeSoc);  //&data_blockrdBmsCumtimeCharge_temp,
        //-------------------------------------------------------------------------
        // Send the data base to the server (PC) via CAN communication
        if (can_rxMessages[14].canNode->IF2DATx[0] == 0xAA) {
            DATABASE_can_transmission();
        }
    }
    //-------------------------------------------------------------------------

    ALGO_MainFunction();
    ftsk_cyclicAlgorithm100msCounter++;
    // record the database inside the flash memory
    if (ftsk_cyclicAlgorithm100msCounter >= 598 && ftsk_cyclicAlgorithm100msCounter <= 600) {
        fp = fopen("SOC_last_value.bin", "wb");
        fwrite(&data_blockECM.SOC_EKF, sizeof(data_blockECM.SOC_EKF), 1, fp);
        fclose(fp);
        DATA_save_BIN_file(select_file);
        select_file++;
        if (select_file == 3) {
            select_file                      = 0;
            ftsk_cyclicAlgorithm100msCounter = 0;
        }
        DATA_WRITE_DATA(&data_blockDatabase);
    }
}

void FTSK_RunUserCodeAfe(void) { /* user code */

#if (FOXBMS_AFE_DRIVER_TYPE_NO_FSM == 1)
    MEAS_Control();
#endif
}

extern void FTSK_RunUserCodeIdle(void) { /* user code */
}

// --------------------------------------------------------------------------------------------------------------------

extern void DATA_read_BIN_file(int select_file) {
    //-------------------------------------------------------------------------
    // 1. Read the corresponding file and update the database
    FILE *fp;
    if (select_file == 0) {
        fp = fopen("Database_Global_time.bin", "rb");
        for (int row = 0; row < 12; row++) {
            if (row < 6) {
                fread(&data_blockrdBmsOpTime.HR[row], sizeof(uint16_t), 1, fp);
            } else if (row < 12) {
                fread(&data_blockrdBmsConsumption.CONSUMPTION[row - 6], sizeof(uint16_t), 1, fp);
            }
        }
        DATA_WRITE_DATA(&data_blockrdBmsOpTime, &data_blockrdBmsConsumption);
    } else if (select_file == 1) {
        fp = fopen("Database_Number_of_times.bin", "rb");
        for (int row = 0; row < 40; row++) {
            if (row < 8) {
                fread(&data_blockrdBmsEoc.EOC[row], sizeof(uint16_t), 1, fp);
            } else if (row < 16) {
                fread(&data_blockrdBmsEod.EOD[row - 8], sizeof(uint16_t), 1, fp);
            } else if (row < 24) {
                fread(&data_blockrdBmsDod.DOD[row - 16], sizeof(uint16_t), 1, fp);
            } else if (row < 32) {
                fread(&data_blockrdBmsDoc.DOC[row - 24], sizeof(uint16_t), 1, fp);
            } else {
                fread(&data_blockrdBmsCumtimeSoc.SOC[row], sizeof(uint16_t), 1, fp);
            }
        }
        DATA_WRITE_DATA(&data_blockrdBmsEoc, &data_blockrdBmsEod, &data_blockrdBmsDod, &data_blockrdBmsDoc);
        DATA_WRITE_DATA(&data_blockrdBmsCumtimeSoc);
    } else {
        fp = fopen("Database_Time_spent.bin", "rb");
        for (int row = 0; row < 40; row++) {
            if (row < 5) {
                fread(&data_blockrdBmsCumtimeDischarge.DCC[row], sizeof(uint16_t), 1, fp);
            } else if (row < 10) {
                fread(&data_blockrdBmsCumtimeCharge.CCC[row - 5], sizeof(uint16_t), 1, fp);
            } else if (row < 15) {
                fread(&data_blockrdBmsCumtimeRegen.REG[row - 10], sizeof(uint16_t), 1, fp);
            } else if (row < 20) {
                fread(&data_blockrdBmsCumtimeOptemp.OPT[row - 15], sizeof(uint16_t), 1, fp);
            } else if (row < 25) {
                fread(&data_blockrdBmsCumtimeCharge_temp.CPT[row - 20], sizeof(uint16_t), 1, fp);
            } else if (row < 30) {
                fread(&data_blockrdBmsCumtimeDischarge_temp.DPT[row - 25], sizeof(uint16_t), 1, fp);
            } else {
                fread(&data_blockrdBmsCumtimeRegen_temp.REGT[row - 30], sizeof(uint16_t), 1, fp);
            }
        }
        DATA_WRITE_DATA(
            &data_blockrdBmsCumtimeDischarge,
            &data_blockrdBmsCumtimeCharge,
            &data_blockrdBmsCumtimeRegen,
            &data_blockrdBmsCumtimeOptemp);
        DATA_WRITE_DATA(&data_blockrdBmsCumtimeDischarge_temp, &data_blockrdBmsCumtimeRegen_temp);
    }
    fclose(fp);
}
// --------------------------------------------------------------------------------------------------------------------

extern void DATA_save_BIN_file(int select_file) {
    // 1. Write in the corresponding file
    FILE *fp;
    if (select_file == 0) {
        fp = fopen("Database_Global_time.bin", "wb");
        fwrite(&data_blockrdBmsOpTime.HR, sizeof(data_blockrdBmsOpTime.HR), 1, fp);
        fwrite(&data_blockrdBmsConsumption.CONSUMPTION, sizeof(data_blockrdBmsConsumption.CONSUMPTION), 1, fp);
    } else if (select_file == 1) {
        fp = fopen("Database_Number_of_times.bin", "wb");
        fwrite(&data_blockrdBmsEoc.EOC, 1, sizeof(data_blockrdBmsEoc.EOC), fp);
        fwrite(&data_blockrdBmsEod.EOD, 1, sizeof(data_blockrdBmsEod.EOD), fp);
        fwrite(&data_blockrdBmsDod.DOD, 1, sizeof(data_blockrdBmsDod.DOD), fp);
        fwrite(&data_blockrdBmsDoc.DOC, 1, sizeof(data_blockrdBmsDoc.DOC), fp);
        fwrite(&data_blockrdBmsCumtimeSoc.SOC, 1, sizeof(data_blockrdBmsCumtimeSoc.SOC), fp);
    } else {
        fp = fopen("Database_Time_spent.bin", "wb");
        fwrite(&data_blockrdBmsCumtimeDischarge.DCC, 1, sizeof(data_blockrdBmsCumtimeDischarge.DCC), fp);
        fwrite(&data_blockrdBmsCumtimeCharge.CCC, 1, sizeof(data_blockrdBmsCumtimeCharge.CCC), fp);
        fwrite(&data_blockrdBmsCumtimeRegen.REG, 1, sizeof(data_blockrdBmsCumtimeRegen.REG), fp);
        fwrite(&data_blockrdBmsCumtimeOptemp.OPT, 1, sizeof(data_blockrdBmsCumtimeOptemp.OPT), fp);
        fwrite(&data_blockrdBmsCumtimeCharge_temp.CPT, 1, sizeof(data_blockrdBmsCumtimeCharge_temp.CPT), fp);
        fwrite(&data_blockrdBmsCumtimeDischarge_temp.DPT, 1, sizeof(data_blockrdBmsCumtimeDischarge_temp.DPT), fp);
        fwrite(&data_blockrdBmsCumtimeRegen_temp.REGT, 1, sizeof(data_blockrdBmsCumtimeRegen_temp.REGT), fp);
    }
    fclose(fp);
}

extern void DATABASE_can_transmission(void) {
    //-------------------------------------------------------------------------
    uint8_t Data_can_send[16] = {
        0,
        0,
        0,
        0,
        (uint8_t)(data_blockrdBmsOpTime.HR[5] >> 8),
        (uint8_t)(data_blockrdBmsOpTime.HR[5]),
        (uint8_t)(data_blockrdBmsOpTime.HR[4] >> 8),
        (uint8_t)(data_blockrdBmsOpTime.HR[4]),
        (uint8_t)(data_blockrdBmsOpTime.HR[3] >> 8),
        (uint8_t)(data_blockrdBmsOpTime.HR[3]),
        (uint8_t)(data_blockrdBmsOpTime.HR[2] >> 8),
        (uint8_t)(data_blockrdBmsOpTime.HR[2]),
        (uint8_t)(data_blockrdBmsOpTime.HR[1] >> 8),
        (uint8_t)(data_blockrdBmsOpTime.HR[1]),
        (uint8_t)(data_blockrdBmsOpTime.HR[0] >> 8),
        (uint8_t)(data_blockrdBmsOpTime.HR[0])};

    CAN_DataSend(CAN_NODE_1, 0x2B0, &Data_can_send[0]);  //688
    CAN_DataSend(CAN_NODE_1, 0x2B1, &Data_can_send[0]);
    //-------------------------------------------------------------------------
    int row = 0;
    for (int i = 0; i < 10; i += 2) {
        Data_can_send[i]     = (uint8_t)(data_blockrdBmsConsumption.CONSUMPTION[row] >> 8);
        Data_can_send[i + 1] = (uint8_t)(data_blockrdBmsConsumption.CONSUMPTION[row]);
        row++;
    }

    CAN_DataSend(CAN_NODE_1, 0x2B2, &Data_can_send[0]);
    CAN_DataSend(CAN_NODE_1, 0x2B3, &Data_can_send[8]);
    //-------------------------------------------------------------------------
    row = 0;
    for (int i = 0; i < 16; i += 2) {
        Data_can_send[i]     = (uint8_t)(data_blockrdBmsEoc.EOC[row] >> 8);
        Data_can_send[i + 1] = (uint8_t)(data_blockrdBmsEoc.EOC[row]);
        row++;
    }

    CAN_DataSend(CAN_NODE_1, 0x2B4, &Data_can_send[0]);
    CAN_DataSend(CAN_NODE_1, 0x2B5, &Data_can_send[8]);
    //-------------------------------------------------------------------------
    row = 0;
    for (int i = 0; i < 16; i += 2) {
        Data_can_send[i]     = (uint8_t)(data_blockrdBmsEod.EOD[row] >> 8);
        Data_can_send[i + 1] = (uint8_t)(data_blockrdBmsEod.EOD[row]);
        row++;
    }
    CAN_DataSend(CAN_NODE_1, 0x2B6, &Data_can_send[0]);
    CAN_DataSend(CAN_NODE_1, 0x2B7, &Data_can_send[8]);
    //-------------------------------------------------------------------------

    row = 0;
    for (int i = 0; i < 16; i += 2) {
        Data_can_send[i]     = (uint8_t)(data_blockrdBmsDod.DOD[row] >> 8);
        Data_can_send[i + 1] = (uint8_t)(data_blockrdBmsDod.DOD[row]);
        row++;
    }
    CAN_DataSend(CAN_NODE_1, 0x2B8, &Data_can_send[0]);
    CAN_DataSend(CAN_NODE_1, 0x2B9, &Data_can_send[8]);
    //-------------------------------------------------------------------------

    row = 0;
    for (int i = 0; i < 16; i += 2) {
        Data_can_send[i]     = (uint8_t)(data_blockrdBmsDoc.DOC[row] >> 8);
        Data_can_send[i + 1] = (uint8_t)(data_blockrdBmsDoc.DOC[row]);
        row++;
    }
    CAN_DataSend(CAN_NODE_1, 0x2BA, &Data_can_send[0]);
    CAN_DataSend(CAN_NODE_1, 0x2BB, &Data_can_send[8]);
    //-------------------------------------------------------------------------

    row = 0;
    for (int i = 0; i < 10; i += 2) {
        Data_can_send[i]     = (uint8_t)(data_blockrdBmsCumtimeDischarge.DCC[row] >> 8);
        Data_can_send[i + 1] = (uint8_t)(data_blockrdBmsCumtimeDischarge.DCC[row]);
        row++;
    }
    CAN_DataSend(CAN_NODE_1, 0x2BC, &Data_can_send[0]);
    CAN_DataSend(CAN_NODE_1, 0x2BD, &Data_can_send[8]);
    //-------------------------------------------------------------------------

    row = 0;
    for (int i = 0; i < 10; i += 2) {
        Data_can_send[i]     = (uint8_t)(data_blockrdBmsCumtimeCharge.CCC[row] >> 8);
        Data_can_send[i + 1] = (uint8_t)(data_blockrdBmsCumtimeCharge.CCC[row]);
        row++;
    }
    CAN_DataSend(CAN_NODE_1, 0x2BE, &Data_can_send[0]);
    CAN_DataSend(CAN_NODE_1, 0x2BF, &Data_can_send[8]);
    //-------------------------------------------------------------------------

    row = 0;
    for (int i = 0; i < 10; i += 2) {
        Data_can_send[i]     = (uint8_t)(data_blockrdBmsCumtimeRegen.REG[row] >> 8);
        Data_can_send[i + 1] = (uint8_t)(data_blockrdBmsCumtimeRegen.REG[row]);
        row++;
    }
    CAN_DataSend(CAN_NODE_1, 0x2C0, &Data_can_send[0]);
    CAN_DataSend(CAN_NODE_1, 0x2C1, &Data_can_send[8]);
    //-------------------------------------------------------------------------

    row = 0;
    for (int i = 0; i < 10; i += 2) {
        Data_can_send[i]     = (uint8_t)(data_blockrdBmsCumtimeOptemp.OPT[row] >> 8);
        Data_can_send[i + 1] = (uint8_t)(data_blockrdBmsCumtimeOptemp.OPT[row]);
        row++;
    }
    CAN_DataSend(CAN_NODE_1, 0x2C2, &Data_can_send[0]);
    CAN_DataSend(CAN_NODE_1, 0x2C3, &Data_can_send[8]);
    //-------------------------------------------------------------------------

    row = 0;
    for (int i = 0; i < 10; i += 2) {
        Data_can_send[i]     = (uint8_t)(data_blockrdBmsCumtimeCharge_temp.CPT[row] >> 8);
        Data_can_send[i + 1] = (uint8_t)(data_blockrdBmsCumtimeCharge_temp.CPT[row]);
        row++;
    }
    CAN_DataSend(CAN_NODE_1, 0x2C4, &Data_can_send[0]);
    CAN_DataSend(CAN_NODE_1, 0x2C5, &Data_can_send[8]);
    //-------------------------------------------------------------------------

    row = 0;
    for (int i = 0; i < 10; i += 2) {
        Data_can_send[i]     = (uint8_t)(data_blockrdBmsCumtimeDischarge_temp.DPT[row] >> 8);
        Data_can_send[i + 1] = (uint8_t)(data_blockrdBmsCumtimeDischarge_temp.DPT[row]);
        row++;
    }
    CAN_DataSend(CAN_NODE_1, 0x2C6, &Data_can_send[0]);
    CAN_DataSend(CAN_NODE_1, 0x2C7, &Data_can_send[8]);
    //-------------------------------------------------------------------------

    row = 0;
    for (int i = 0; i < 10; i += 2) {
        Data_can_send[i]     = (uint8_t)(data_blockrdBmsCumtimeRegen_temp.REGT[row] >> 8);
        Data_can_send[i + 1] = (uint8_t)(data_blockrdBmsCumtimeRegen_temp.REGT[row]);
        row++;
    }
    CAN_DataSend(CAN_NODE_1, 0x2C8, &Data_can_send[0]);
    CAN_DataSend(CAN_NODE_1, 0x2C9, &Data_can_send[8]);
    //-------------------------------------------------------------------------

    row = 0;
    for (int i = 0; i < 16; i += 2) {
        Data_can_send[i]     = (uint8_t)(data_blockrdBmsCumtimeSoc.SOC[row] >> 8);
        Data_can_send[i + 1] = (uint8_t)(data_blockrdBmsCumtimeSoc.SOC[row]);
        row++;
    }
    CAN_DataSend(CAN_NODE_1, 0x2CA, &Data_can_send[0]);
    CAN_DataSend(CAN_NODE_1, 0x2CB, &Data_can_send[8]);  //715
    //-------------------------------------------------------------------------
}
/*========== Externalized Static Function Implementations (Unit Test) =======*/
