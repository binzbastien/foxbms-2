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
 * @file    database_cfg.h
 * @author  foxBMS Team
 * @date    2015-08-18 (date of creation)
 * @updated 2022-10-27 (date of last update)
 * @version v1.4.1
 * @ingroup ENGINE_CONFIGURATION
 * @prefix  DATA
 *
 * @brief   Database configuration header
 *
 * @details Provides interfaces to database configuration
 *
 */

#ifndef FOXBMS__DATABASE_CFG_H_
#define FOXBMS__DATABASE_CFG_H_

/*========== Includes =======================================================*/
#include "general.h"

#include "battery_system_cfg.h"

#include "mcu.h"

/*========== Macros and Definitions =========================================*/
/** configuration struct of database channel (data block) */
typedef struct {
    void *pDatabaseEntry; /*!< pointer to the database entry */
    uint32_t dataLength;  /*!< length of the entry */
} DATA_BASE_s;

/** data block identification numbers */
typedef enum {
    DATA_BLOCK_ID_CELL_VOLTAGE,
    DATA_BLOCK_ID_CELL_TEMPERATURE,
    DATA_BLOCK_ID_MIN_MAX,
    DATA_BLOCK_ID_CURRENT_SENSOR,
    DATA_BLOCK_ID_BALANCING_CONTROL,
    DATA_BLOCK_ID_SLAVE_CONTROL,
    DATA_BLOCK_ID_BALANCING_FEEDBACK_BASE,
    DATA_BLOCK_ID_USER_MUX,
    DATA_BLOCK_ID_OPEN_WIRE_BASE,
    DATA_BLOCK_ID_ALL_GPIO_VOLTAGES_BASE,
    DATA_BLOCK_ID_ERRORSTATE,
    DATA_BLOCK_ID_CONTACTOR_FEEDBACK,
    DATA_BLOCK_ID_INTERLOCK_FEEDBACK,
    DATA_BLOCK_ID_SOF,
    DATA_BLOCK_ID_SYSTEMSTATE,
    DATA_BLOCK_ID_MSL_FLAG,
    DATA_BLOCK_ID_RSL_FLAG,
    DATA_BLOCK_ID_MOL_FLAG,
    DATA_BLOCK_ID_SOX,
    DATA_BLOCK_ID_STATEREQUEST,
    DATA_BLOCK_ID_MOVING_AVERAGE,
    DATA_BLOCK_ID_CELL_VOLTAGE_BASE,
    DATA_BLOCK_ID_CELL_TEMPERATURE_BASE,
    DATA_BLOCK_ID_CELL_VOLTAGE_REDUNDANCY0,
    DATA_BLOCK_ID_CELL_TEMPERATURE_REDUNDANCY0,
    DATA_BLOCK_ID_BALANCING_FEEDBACK_REDUNDANCY0,
    DATA_BLOCK_ID_ALL_GPIO_VOLTAGES_REDUNDANCY0,
    DATA_BLOCK_ID_OPEN_WIRE_REDUNDANCY0,
    DATA_BLOCK_ID_INSULATION_MONITORING,
    DATA_BLOCK_ID_PACK_VALUES,
    DATA_BLOCK_ID_HTSEN,
    DATA_BLOCK_ID_ADC_VOLTAGE,
    DATA_BLOCK_ID_DUMMY_FOR_SELF_TEST,
    DATA_BLOCK_ID_DATABASE,
    DATA_BLOCK_ID_EKF,
    DATA_BLOCK_ID_ECM,
    DATA_BLOCK_ID_RD_BMS_OP_TIME,
    DATA_BLOCK_ID_RD_BMS_CONSUMPTION,
    DATA_BLOCK_ID_RD_BMS_EOC,
    DATA_BLOCK_ID_RD_BMS_EOD,
    DATA_BLOCK_ID_RD_BMS_DOD,
    DATA_BLOCK_ID_RD_BMS_DOC,
    DATA_BLOCK_ID_RD_BMS_CUMTIME_DISCHARGE,
    DATA_BLOCK_ID_RD_BMS_CUMTIME_CHARGE,
    DATA_BLOCK_ID_RD_BMS_CUMTIME_REGEN,
    DATA_BLOCK_ID_RD_BMS_CUMTIME_OPTEMP,
    DATA_BLOCK_ID_RD_BMS_CUMTIME_CHARGE_TEMP,
    DATA_BLOCK_ID_RD_BMS_CUMTIME_DISCHARGE_TEMP,
    DATA_BLOCK_ID_RD_BMS_CUMTIME_REGEN_TEMP,
    DATA_BLOCK_ID_RD_BMS_CUMTIME_SOC,
    DATA_BLOCK_ID_MAX, /**< DO NOT CHANGE, MUST BE THE LAST ENTRY */
} DATA_BLOCK_ID_e;

FAS_STATIC_ASSERT(
    (int16_t)DATA_BLOCK_ID_MAX < UINT8_MAX,
    "Maximum number of database entries exceeds UINT8_MAX; adapted length "
    "checking in DATA_Initialize and DATA_IterateOverDatabaseEntries");

/** data block header */
typedef struct {
    DATA_BLOCK_ID_e uniqueId;   /*!< uniqueId of database entry */
    uint32_t timestamp;         /*!< timestamp of last database update */
    uint32_t previousTimestamp; /*!< timestamp of previous database update */
} DATA_BLOCK_HEADER_s;

/** data block struct of cell voltage */
typedef struct {
    /* This struct needs to be at the beginning of every database entry. During
     * the initialization of a database struct, uniqueId must be set to the
     * respective database entry representation in enum DATA_BLOCK_ID_e. */
    DATA_BLOCK_HEADER_s header;                                                /*!< Data block header */
    uint8_t state;                                                             /*!< for future use */
    int32_t packVoltage_mV[BS_NR_OF_STRINGS];                                  /*!< uint: mV */
    int16_t cellVoltage_mV[BS_NR_OF_STRINGS][BS_NR_OF_CELL_BLOCKS_PER_STRING]; /*!< unit: mV */
    uint64_t
        invalidCellVoltage[BS_NR_OF_STRINGS]
                          [BS_NR_OF_MODULES_PER_STRING]; /*!< bitmask if voltages are valid. 0->valid, 1->invalid */
    uint16_t nrValidCellVoltages[BS_NR_OF_STRINGS];      /*!< number of valid voltages */
    uint32_t moduleVoltage_mV[BS_NR_OF_STRINGS][BS_NR_OF_MODULES_PER_STRING]; /*!< unit: mV */
    bool validModuleVoltage[BS_NR_OF_STRINGS][BS_NR_OF_MODULES_PER_STRING];   /*!< 0 -> if PEC okay; 1 -> PEC error */
} DATA_BLOCK_CELL_VOLTAGE_s;

/** data block struct of cell temperatures */
typedef struct {
    /* This struct needs to be at the beginning of every database entry. During
     * the initialization of a database struct, uniqueId must be set to the
     * respective database entry representation in enum DATA_BLOCK_ID_e. */
    DATA_BLOCK_HEADER_s header;                                                        /*!< Data block header */
    uint8_t state;                                                                     /*!< for future use */
    int16_t cellTemperature_ddegC[BS_NR_OF_STRINGS][BS_NR_OF_TEMP_SENSORS_PER_STRING]; /*!< unit: deci &deg;C */
    uint16_t invalidCellTemperature
        [BS_NR_OF_STRINGS][BS_NR_OF_MODULES_PER_STRING]; /*!< bitmask if temperatures are valid. 0->valid, 1->invalid */
    uint16_t nrValidTemperatures[BS_NR_OF_STRINGS];      /*!< number of valid temperatures in each string */
} DATA_BLOCK_CELL_TEMPERATURE_s;

/** data block struct of minimum and maximum values */
typedef struct {
    /* This struct needs to be at the beginning of every database entry. During
     * the initialization of a database struct, uniqueId must be set to the
     * respective database entry representation in enum DATA_BLOCK_ID_e. */
    DATA_BLOCK_HEADER_s header; /*!< Data block header */

    int16_t averageCellVoltage_mV[BS_NR_OF_STRINGS];          /*!< average cell voltages, unit: mV */
    int16_t minimumCellVoltage_mV[BS_NR_OF_STRINGS];          /*!< minimum cell voltages, unit: mV */
    int16_t previousMinimumCellVoltage_mV[BS_NR_OF_STRINGS];  /*!< previous minimum cell voltages, unit: mV */
    int16_t maximumCellVoltage_mV[BS_NR_OF_STRINGS];          /*!< maximum cell voltages, unit: mV */
    int16_t previousMaximumCellVoltage_mV[BS_NR_OF_STRINGS];  /*!< previous maximum cell voltages, unit: mV */
    uint16_t nrModuleMinimumCellVoltage[BS_NR_OF_STRINGS];    /*!< number of the module with minimum cell voltage */
    uint16_t nrCellMinimumCellVoltage[BS_NR_OF_STRINGS];      /*!< number of the cell with minimum cell voltage */
    uint16_t nrModuleMaximumCellVoltage[BS_NR_OF_STRINGS];    /*!< number of the module with maximum cell voltage */
    uint16_t nrCellMaximumCellVoltage[BS_NR_OF_STRINGS];      /*!< number of the cell with maximum cell voltage */
    uint16_t validMeasuredCellVoltages[BS_NR_OF_STRINGS];     /*!< number of valid measured cell voltages */
    float averageTemperature_ddegC[BS_NR_OF_STRINGS];         /*!< unit: deci &deg;C */
    int16_t minimumTemperature_ddegC[BS_NR_OF_STRINGS];       /*!< unit: deci &deg;C */
    uint16_t nrModuleMinimumTemperature[BS_NR_OF_STRINGS];    /*!< number of the module with minimum temperature */
    uint16_t nrSensorMinimumTemperature[BS_NR_OF_STRINGS];    /*!< number of the sensor with minimum temperature */
    int16_t maximumTemperature_ddegC[BS_NR_OF_STRINGS];       /*!< unit: deci &deg;C */
    uint16_t nrModuleMaximumTemperature[BS_NR_OF_STRINGS];    /*!< number of the module with maximum temperature */
    uint16_t nrSensorMaximumTemperature[BS_NR_OF_STRINGS];    /*!< number of the sensor with maximum temperature */
    uint16_t validMeasuredCellTemperatures[BS_NR_OF_STRINGS]; /*!< number of valid measured cell temperatures */
    uint8_t state;                                            /*!< state of the min max module */
} DATA_BLOCK_MIN_MAX_s;

/** data block struct of pack measurement values */
typedef struct {
    /* This struct needs to be at the beginning of every database entry. During
     * the initialization of a database struct, uniqueId must be set to the
     * respective database entry representation in enum DATA_BLOCK_ID_e. */
    DATA_BLOCK_HEADER_s header; /*!< Data block header */

    int32_t packCurrent_mA;        /*!< current in the whole battery pack, unit: mA */
    uint8_t invalidPackCurrent;    /*!< bitmask if current is valid. 0->valid, 1->invalid */
    int32_t batteryVoltage_mV;     /*!< voltage between negative and positive battery pole, unit: mV */
    uint8_t invalidBatteryVoltage; /*!< bitmask if voltage is valid. 0->valid, 1->invalid */
    int32_t
        highVoltageBusVoltage_mV; /*!< voltage between negative battery pole and after positive main contactor, unit: mV */
    uint8_t invalidHvBusVoltage; /*!< bitmask if voltage is valid. 0->valid, 1->invalid */
    int32_t packPower_W;         /*!< power provided by respectively supplied to the battery pack, unit: W */
    uint8_t invalidPackPower;    /*!< bitmask if power is valid. 0->valid, 1->invalid */
    int32_t stringVoltage_mV[BS_NR_OF_STRINGS];     /*!< voltage of each string, unit: mV */
    uint8_t invalidStringVoltage[BS_NR_OF_STRINGS]; /*!< bitmask if voltages are valid. 0->valid, 1->invalid */
    int32_t stringCurrent_mA[BS_NR_OF_STRINGS];     /*!< current in each string, unit: mA */
    uint8_t invalidStringCurrent[BS_NR_OF_STRINGS]; /*!< bitmask if currents are valid. 0->valid, 1->invalid */
    int32_t stringPower_W[BS_NR_OF_STRINGS];        /*!< power of each string, unit: W */
    uint8_t invalidStringPower[BS_NR_OF_STRINGS];   /*!< bitmask if power values are valid. 0->valid, 1->invalid */
} DATA_BLOCK_PACK_VALUES_s;

/** data block struct of current measurement */
typedef struct {
    /* This struct needs to be at the beginning of every database entry. During
     * the initialization of a database struct, uniqueId must be set to the
     * respective database entry representation in enum DATA_BLOCK_ID_e. */
    DATA_BLOCK_HEADER_s header;                                    /*!< Data block header */
    int32_t current_mA[BS_NR_OF_STRINGS];                          /*!< unit: mA */
    uint8_t invalidCurrentMeasurement[BS_NR_OF_STRINGS];           /*!< 0: measurement valid, 1: measurement invalid */
    uint8_t newCurrent;                                            /*!< 0: measurement valid, 1: measurement invalid */
    uint32_t previousTimestampCurrent[BS_NR_OF_STRINGS];           /*!< timestamp of current measurement */
    uint32_t timestampCurrent[BS_NR_OF_STRINGS];                   /*!< timestamp of current measurement */
    int32_t sensorTemperature_ddegC[BS_NR_OF_STRINGS];             /*!< unit: 0.1&deg;C */
    uint8_t invalidSensorTemperatureMeasurement[BS_NR_OF_STRINGS]; /*!< 0: measurement valid, 1: measurement invalid */
    int32_t power_W[BS_NR_OF_STRINGS];                             /*!< unit: W */
    uint8_t invalidPowerMeasurement[BS_NR_OF_STRINGS];             /*!< 0: measurement valid, 1: measurement invalid */
    uint8_t newPower;                                            /*!< counter that indicates a new power measurement */
    uint32_t previousTimestampPower[BS_NR_OF_STRINGS];           /*!< previous timestamp of power measurement */
    uint32_t timestampPower[BS_NR_OF_STRINGS];                   /*!< timestamp of power measurement */
    int32_t currentCounter_As[BS_NR_OF_STRINGS];                 /*!< unit: A.s */
    uint8_t invalidCurrentCountingMeasurement[BS_NR_OF_STRINGS]; /*!< 0: measurement valid, 1: measurement invalid */
    uint32_t previousTimestampCurrentCounting[BS_NR_OF_STRINGS]; /*!< previous timestamp of CC measurement */
    uint32_t timestampCurrentCounting[BS_NR_OF_STRINGS];         /*!< timestamp of CC measurement */
    int32_t energyCounter_Wh[BS_NR_OF_STRINGS];                  /*!< unit: Wh */
    uint8_t invalidEnergyCountingMeasurement[BS_NR_OF_STRINGS];  /*!< 0: measurement valid, 1: measurement invalid */
    uint32_t previousTimestampEnergyCounting[BS_NR_OF_STRINGS];  /*!< previous timestamp of EC measurement */
    uint32_t timestampEnergyCounting[BS_NR_OF_STRINGS];          /*!< timestamp of EC measurement */
    uint8_t invalidHighVoltageMeasurement
        [BS_NR_OF_STRINGS][BS_NR_OF_VOLTAGES_FROM_CURRENT_SENSOR]; /*!< 0: measurement valid, 1: measurement invalid */
    int32_t highVoltage_mV[BS_NR_OF_STRINGS][BS_NR_OF_VOLTAGES_FROM_CURRENT_SENSOR]; /*!< unit: mV */
    uint32_t previousTimestampHighVoltage
        [BS_NR_OF_STRINGS]
        [BS_NR_OF_VOLTAGES_FROM_CURRENT_SENSOR]; /*!< previous timestamp of high voltage measurement */
    uint32_t timestampHighVoltage[BS_NR_OF_STRINGS]
                                 [BS_NR_OF_VOLTAGES_FROM_CURRENT_SENSOR]; /*!< timestamp of high voltage measurement */
} DATA_BLOCK_CURRENT_SENSOR_s;

/** data structure declaration of DATA_BLOCK_BALANCING_CONTROL */
typedef struct {
    /* This struct needs to be at the beginning of every database entry. During
     * the initialization of a database struct, uniqueId must be set to the
     * respective database entry representation in enum DATA_BLOCK_ID_e. */
    DATA_BLOCK_HEADER_s header; /*!< Data block header */
    uint8_t enableBalancing;    /*!< Switch for enabling/disabling balancing  */
    uint8_t threshold_mV;       /*!< balancing threshold in mV                */
    uint8_t request;            /*!< balancing request per CAN                */
    uint8_t balancingState[BS_NR_OF_STRINGS]
                          [BS_NR_OF_CELL_BLOCKS_PER_STRING]; /*!< 0: no balancing, 1: balancing active     */
    uint32_t deltaCharge_mAs[BS_NR_OF_STRINGS]
                            [BS_NR_OF_CELL_BLOCKS_PER_STRING]; /*!< Difference in Depth-of-Discharge in mAs  */
    uint16_t nrBalancedCells[BS_NR_OF_STRINGS];
} DATA_BLOCK_BALANCING_CONTROL_s;

/** data structure declaration of DATA_BLOCK_USER_IO_CONTROL */
typedef struct {
    /* This struct needs to be at the beginning of every database entry. During
     * the initialization of a database struct, uniqueId must be set to the
     * respective database entry representation in enum DATA_BLOCK_ID_e. */
    DATA_BLOCK_HEADER_s header;                            /*!< Data block header */
    uint8_t state;                                         /*!< for future use */
    uint32_t eepromReadAddressToUse;                       /*!< address to read from for  slave EEPROM */
    uint32_t eepromReadAddressLastUsed;                    /*!< last address used to read fromfor slave EEPROM */
    uint32_t eepromWriteAddressToUse;                      /*!< address to write to for slave EEPROM */
    uint32_t eepromWriteAddressLastUsed;                   /*!< last address used to write to for slave EEPROM */
    uint8_t ioValueOut[BS_NR_OF_MODULES_PER_STRING];       /*!< data to be written to the port expander */
    uint8_t ioValueIn[BS_NR_OF_MODULES_PER_STRING];        /*!< data read from to the port expander */
    uint8_t eepromValueWrite[BS_NR_OF_MODULES_PER_STRING]; /*!< data to be written to the slave EEPROM */
    uint8_t eepromValueRead[BS_NR_OF_MODULES_PER_STRING];  /*!< data read from to the slave EEPROM */
    uint8_t
        externalTemperatureSensor[BS_NR_OF_MODULES_PER_STRING]; /*!< temperature from the external sensor on slave */
} DATA_BLOCK_SLAVE_CONTROL_s;

/** data block struct of cell balancing feedback */
typedef struct {
    /* This struct needs to be at the beginning of every database entry. During
     * the initialization of a database struct, uniqueId must be set to the
     * respective database entry representation in enum DATA_BLOCK_ID_e. */
    DATA_BLOCK_HEADER_s header;                                    /*!< Data block header */
    uint8_t state;                                                 /*!< for future use */
    uint16_t value[BS_NR_OF_STRINGS][BS_NR_OF_MODULES_PER_STRING]; /*!< unit: mV (optocoupler output) */
} DATA_BLOCK_BALANCING_FEEDBACK_s;

/** data block struct of user multiplexer values */
typedef struct {
    /* This struct needs to be at the beginning of every database entry. During
     * the initialization of a database struct, uniqueId must be set to the
     * respective database entry representation in enum DATA_BLOCK_ID_e. */
    DATA_BLOCK_HEADER_s header;                                              /*!< Data block header */
    uint8_t state;                                                           /*!< for future use */
    uint16_t value[BS_NR_OF_STRINGS][8u * 2u * BS_NR_OF_MODULES_PER_STRING]; /*!< unit: mV (mux voltage input) */
} DATA_BLOCK_USER_MUX_s;

/** data block struct of cell open wire */
typedef struct {
    /* This struct needs to be at the beginning of every database entry. During
     * the initialization of a database struct, uniqueId must be set to the
     * respective database entry representation in enum DATA_BLOCK_ID_e. */
    DATA_BLOCK_HEADER_s header;             /*!< Data block header */
    uint8_t state;                          /*!< for future use */
    uint16_t nrOpenWires[BS_NR_OF_STRINGS]; /*!< number of open wires */
    uint8_t openwire[BS_NR_OF_STRINGS]
                    [BS_NR_OF_MODULES_PER_STRING *
                     (BS_NR_OF_CELL_BLOCKS_PER_MODULE + 1u)]; /*!< 1 -> open wire, 0 -> everything ok */
} DATA_BLOCK_OPEN_WIRE_s;

/** data block struct of GPIO voltage */
typedef struct {
    /* This struct needs to be at the beginning of every database entry. During
     * the initialization of a database struct, uniqueId must be set to the
     * respective database entry representation in enum DATA_BLOCK_ID_e. */
    DATA_BLOCK_HEADER_s header; /*!< Data block header */
    uint8_t state;              /*!< for future use */
    uint16_t gpioVoltages_mV[BS_NR_OF_STRINGS]
                            [BS_NR_OF_MODULES_PER_STRING * BS_NR_OF_GPIOS_PER_MODULE]; /*!< unit: mV */
    uint16_t
        invalidGpioVoltages[BS_NR_OF_STRINGS]
                           [BS_NR_OF_MODULES_PER_STRING]; /*!< bitmask if voltages are valid. 0->valid, 1->invalid */
} DATA_BLOCK_ALL_GPIO_VOLTAGES_s;

/** data block struct of error flags */
typedef struct {
    /* This struct needs to be at the beginning of every database entry. During
     * the initialization of a database struct, uniqueId must be set to the
     * respective database entry representation in enum DATA_BLOCK_ID_e. */
    DATA_BLOCK_HEADER_s header;                           /*!< Data block header */
    uint8_t currentSensor[BS_NR_OF_STRINGS];              /*!< 0 -> no error, 1 -> error, not responding */
    uint8_t stringMinusContactor[BS_NR_OF_STRINGS];       /*!< 0 -> no error, 1 -> error, not responding */
    uint8_t stringPlusContactor[BS_NR_OF_STRINGS];        /*!< 0 -> no error, 1 -> error, not responding */
    uint8_t prechargeContactor[BS_NR_OF_STRINGS];         /*!< 0 -> no error, 1 -> error, not responding */
    uint8_t interlock;                                    /*!< 0 -> no error, 1 -> error */
    uint8_t crcError[BS_NR_OF_STRINGS];                   /*!< 0 -> no error, 1 -> error */
    uint8_t muxError[BS_NR_OF_STRINGS];                   /*!< 0 -> no error, 1 -> error */
    uint8_t spiError[BS_NR_OF_STRINGS];                   /*!< 0 -> no error, 1 -> error */
    uint8_t afeConfigurationError[BS_NR_OF_STRINGS];      /*!< 0 -> no error, 1 -> error */
    uint8_t afeCellVoltageError[BS_NR_OF_STRINGS];        /*!< 0 -> no error, 1 -> error */
    uint8_t afeCellTemperatureError[BS_NR_OF_STRINGS];    /*!< 0 -> no error, 1 -> error */
    uint8_t baseCellVoltageMeasurementTimeout;            /*!< 0 -> no error, 1 -> error */
    uint8_t redundancy0CellVoltageMeasurementTimeout;     /*!< 0 -> no error, 1 -> error */
    uint8_t baseCellTemperatureMeasurementTimeout;        /*!< 0 -> no error, 1 -> error */
    uint8_t redundancy0CellTemperatureMeasurementTimeout; /*!< 0 -> no error, 1 -> error */
    uint8_t currentMeasurementTimeout[BS_NR_OF_STRINGS];  /*!< 0 -> no error, 1 -> error */
    uint8_t currentMeasurementError[BS_NR_OF_STRINGS];    /*!< 0 -> no error, 1 -> error */
    uint8_t currentSensorTimeoutV1[BS_NR_OF_STRINGS];     /*!< 0 -> no error, 1 -> error */
    uint8_t currentSensorTimeoutV3[BS_NR_OF_STRINGS];     /*!< 0 -> no error, 1 -> error */
    uint8_t currentSensorPowerTimeout[BS_NR_OF_STRINGS];  /*!< 0 -> no error, 1 -> error */
    uint8_t powerMeasurementError[BS_NR_OF_STRINGS];      /*!< 0 -> no error, 1 -> error */
    bool insulationMeasurementValid;                      /*!< false -> not valid, true -> valid */
    bool
        criticalLowInsulationResistance; /*!< false -> no critical resistance measured, true -> critical low resistance measured */
    bool
        warnableLowInsulationResistance; /*!< false -> no warnable resistance measured, true -> warnable low resistance measured */
    bool
        insulationGroundFaultDetected; /*!< false -> no insulation fault between HV and chassis detected, true -> insulation fault detected */
    uint8_t fuseStateNormal[BS_NR_OF_STRINGS];                        /*!< 0 -> fuse ok,  1 -> fuse tripped */
    uint8_t fuseStateCharge[BS_NR_OF_STRINGS];                        /*!< 0 -> fuse ok,  1 -> fuse tripped */
    uint8_t open_wire[BS_NR_OF_STRINGS];                              /*!< 0 -> no error, 1 -> error */
    uint8_t canTiming;                                                /*!< 0 -> no error, 1 -> error */
    uint8_t canRxQueueFull;                                           /*!< 0 -> no error, 1 -> error */
    uint8_t canTimingCc[BS_NR_OF_STRINGS];                            /*!< 0 -> no error, 1 -> error */
    uint8_t canTimingEc[BS_NR_OF_STRINGS];                            /*!< 0 -> no error, 1 -> error */
    uint8_t mcuDieTemperature;                                        /*!< 0 -> no error, 1 -> error */
    uint8_t coinCellVoltage;                                          /*!< 0 -> no error, 1 -> error */
    uint8_t plausibilityCheckPackvoltage[BS_NR_OF_STRINGS];           /*!< 0 -> no error, else: error */
    uint8_t plausibilityCheckCellVoltage[BS_NR_OF_STRINGS];           /*!< 0 -> no error, else: error */
    uint8_t plausibilityCheckCellVoltageSpread[BS_NR_OF_STRINGS];     /*!< 0 -> no error, else: error */
    uint8_t plausibilityCheckCelltemperatureSpread[BS_NR_OF_STRINGS]; /*!< 0 -> no error, 1 -> error */
    uint8_t plausibilityCheckCelltemperature[BS_NR_OF_STRINGS];       /*!< 0 -> no error, else: error */
    uint8_t deepDischargeDetected[BS_NR_OF_STRINGS];                  /*!< 0 -> no error, 1 -> error */
    uint8_t currentOnOpenString[BS_NR_OF_STRINGS];                    /*!< 0 -> no error, 1 -> error */
    uint8_t sbcFinState;           /*!< 0 -> okay, 1 -> error: short-circuit to RSTB */
    uint8_t sbcRstbState;          /*!< 0 -> okay, 1 -> error: RSTB not working */
    uint8_t i2cPexError;           /*!< the I2C port expander does not work as expected */
    uint8_t framReadCrcError;      /*!< 0 if read CRC matches with CRC of read data , 1 otherwise */
    bool timingViolationEngine;    /*!< timing violation in engine task */
    bool timingViolation1ms;       /*!< timing violation in 1ms task */
    bool timingViolation10ms;      /*!< timing violation in 10ms task */
    bool timingViolation100ms;     /*!< timing violation in 100ms task */
    bool timingViolation100msAlgo; /*!< timing violation in 100ms algorithm task */
    bool alertFlag;                /*!< true: ALERT situation detected, false: everything okay */
} DATA_BLOCK_ERRORSTATE_s;

/** data block struct of contactor feedback */
typedef struct {
    /* This struct needs to be at the beginning of every database entry. During
     * the initialization of a database struct, uniqueId must be set to the
     * respective database entry representation in enum DATA_BLOCK_ID_e. */
    DATA_BLOCK_HEADER_s header; /*!< Data block header */
    uint32_t contactorFeedback; /*!< feedback of all contactors, without interlock */
} DATA_BLOCK_CONTACTOR_FEEDBACK_s;

/** data block struct of interlock feedback */
typedef struct {
    /* This struct needs to be at the beginning of every database entry. During
     * the initialization of a database struct, uniqueId must be set to the
     * respective database entry representation in enum DATA_BLOCK_ID_e. */
    DATA_BLOCK_HEADER_s header;                 /*!< Data block header */
    uint8_t interlockFeedback_IL_STATE;         /*!< feedback of interlock, connected to pin */
    float interlockVoltageFeedback_IL_HS_VS_mV; /*!< voltage feedback of interlock, connected to ADC input 2 */
    float interlockVoltageFeedback_IL_LS_VS_mV; /*!< voltage feedback of interlock, connected to ADC input 3 */
    float interlockCurrentFeedback_IL_HS_CS_mA; /*!< current feedback of interlock, connected to ADC input 4 */
    float interlockCurrentFeedback_IL_LS_CS_mA; /*!< current feedback of interlock, connected to ADC input 5 */
} DATA_BLOCK_INTERLOCK_FEEDBACK_s;

/** data block struct of sof limits */
typedef struct {
    /* This struct needs to be at the beginning of every database entry. During
     * the initialization of a database struct, uniqueId must be set to the
     * respective database entry representation in enum DATA_BLOCK_ID_e. */
    DATA_BLOCK_HEADER_s header;                         /*!< Data block header */
    float recommendedContinuousPackChargeCurrent_mA;    /*!< recommended continuous operating pack charge current */
    float recommendedContinuousPackDischargeCurrent_mA; /*!< recommended continuous operating pack discharge current */
    float recommendedPeakPackChargeCurrent_mA;          /*!< recommended peak operating pack charge current */
    float recommendedPeakPackDischargeCurrent_mA;       /*!< recommended peak operating pack discharge current */
    float recommendedContinuousChargeCurrent_mA
        [BS_NR_OF_STRINGS]; /*!< recommended continuous operating charge current    */
    float recommendedContinuousDischargeCurrent_mA
        [BS_NR_OF_STRINGS];                                  /*!< recommended continuous operating discharge current */
    float recommendedPeakChargeCurrent_mA[BS_NR_OF_STRINGS]; /*!< recommended peak operating charge current */
    float recommendedPeakDischargeCurrent_mA[BS_NR_OF_STRINGS]; /*!< recommended peak operating discharge current */
} DATA_BLOCK_SOF_s;

/** data block struct of system state */
typedef struct {
    /* This struct needs to be at the beginning of every database entry. During
     * the initialization of a database struct, uniqueId must be set to the
     * respective database entry representation in enum DATA_BLOCK_ID_e. */
    DATA_BLOCK_HEADER_s header; /*!< Data block header */
    int32_t bmsCanState;        /*!< system state for CAN messages (e.g., standby, normal) */
} DATA_BLOCK_SYSTEMSTATE_s;

/** data block struct of the maximum safe limits */
typedef struct {
    /* This struct needs to be at the beginning of every database entry. During
     * the initialization of a database struct, uniqueId must be set to the
     * respective database entry representation in enum DATA_BLOCK_ID_e. */
    DATA_BLOCK_HEADER_s header;                           /*!< Data block header */
    uint8_t packChargeOvercurrent;                        /*!< 0 -> MSL NOT violated, 1 -> MSL violated */
    uint8_t packDischargeOvercurrent;                     /*!< 0 -> MSL NOT violated, 1 -> MSL violated */
    uint8_t overVoltage[BS_NR_OF_STRINGS];                /*!< 0 -> MSL NOT violated, 1 -> MSL violated */
    uint8_t underVoltage[BS_NR_OF_STRINGS];               /*!< 0 -> MSL NOT violated, 1 -> MSL violated */
    uint8_t overtemperatureCharge[BS_NR_OF_STRINGS];      /*!< 0 -> MSL NOT violated, 1 -> MSL violated */
    uint8_t overtemperatureDischarge[BS_NR_OF_STRINGS];   /*!< 0 -> MSL NOT violated, 1 -> MSL violated */
    uint8_t undertemperatureCharge[BS_NR_OF_STRINGS];     /*!< 0 -> MSL NOT violated, 1 -> MSL violated */
    uint8_t undertemperatureDischarge[BS_NR_OF_STRINGS];  /*!< 0 -> MSL NOT violated, 1 -> MSL violated */
    uint8_t cellChargeOvercurrent[BS_NR_OF_STRINGS];      /*!< 0 -> MSL NOT violated, 1 -> MSL violated */
    uint8_t stringChargeOvercurrent[BS_NR_OF_STRINGS];    /*!< 0 -> MSL NOT violated, 1 -> MSL violated */
    uint8_t cellDischargeOvercurrent[BS_NR_OF_STRINGS];   /*!< 0 -> MSL NOT violated, 1 -> MSL violated */
    uint8_t stringDischargeOvercurrent[BS_NR_OF_STRINGS]; /*!< 0 -> MSL NOT violated, 1 -> MSL violated */
    uint8_t pcbOvertemperature[BS_NR_OF_STRINGS];         /*!< 0 -> MSL NOT violated, 1 -> MSL violated */
    uint8_t pcbUndertemperature[BS_NR_OF_STRINGS];        /*!< 0 -> MSL NOT violated, 1 -> MSL violated */
} DATA_BLOCK_MSL_FLAG_s;

/** data block struct of the recommended safety limit */
typedef struct {
    /* This struct needs to be at the beginning of every database entry. During
     * the initialization of a database struct, uniqueId must be set to the
     * respective database entry representation in enum DATA_BLOCK_ID_e. */
    DATA_BLOCK_HEADER_s header;                           /*!< Data block header */
    uint8_t overVoltage[BS_NR_OF_STRINGS];                /*!< 0 -> RSL NOT violated, 1 -> RSL violated */
    uint8_t underVoltage[BS_NR_OF_STRINGS];               /*!< 0 -> RSL NOT violated, 1 -> RSL violated */
    uint8_t overtemperatureCharge[BS_NR_OF_STRINGS];      /*!< 0 -> RSL NOT violated, 1 -> RSL violated */
    uint8_t overtemperatureDischarge[BS_NR_OF_STRINGS];   /*!< 0 -> RSL NOT violated, 1 -> RSL violated */
    uint8_t undertemperatureCharge[BS_NR_OF_STRINGS];     /*!< 0 -> RSL NOT violated, 1 -> RSL violated */
    uint8_t undertemperatureDischarge[BS_NR_OF_STRINGS];  /*!< 0 -> RSL NOT violated, 1 -> RSL violated */
    uint8_t cellChargeOvercurrent[BS_NR_OF_STRINGS];      /*!< 0 -> RSL NOT violated, 1 -> RSL violated */
    uint8_t stringChargeOvercurrent[BS_NR_OF_STRINGS];    /*!< 0 -> RSL NOT violated, 1 -> RSL violated */
    uint8_t cellDischargeOvercurrent[BS_NR_OF_STRINGS];   /*!< 0 -> RSL NOT violated, 1 -> RSL violated */
    uint8_t stringDischargeOvercurrent[BS_NR_OF_STRINGS]; /*!< 0 -> RSL NOT violated, 1 -> RSL violated */
    uint8_t pcbOvertemperature[BS_NR_OF_STRINGS];         /*!< 0 -> RSL NOT violated, 1 -> RSL violated */
    uint8_t pcbUndertemperature[BS_NR_OF_STRINGS];        /*!< 0 -> RSL NOT violated, 1 -> RSL violated */
} DATA_BLOCK_RSL_FLAG_s;

/** data block struct of the maximum operating limit */
typedef struct {
    /* This struct needs to be at the beginning of every database entry. During
     * the initialization of a database struct, uniqueId must be set to the
     * respective database entry representation in enum DATA_BLOCK_ID_e. */
    DATA_BLOCK_HEADER_s header;                           /*!< Data block header */
    uint8_t overVoltage[BS_NR_OF_STRINGS];                /*!< 0 -> MOL NOT violated, 1 -> MOL violated */
    uint8_t underVoltage[BS_NR_OF_STRINGS];               /*!< 0 -> MOL NOT violated, 1 -> MOL violated */
    uint8_t overtemperatureCharge[BS_NR_OF_STRINGS];      /*!< 0 -> MOL NOT violated, 1 -> MOL violated */
    uint8_t overtemperatureDischarge[BS_NR_OF_STRINGS];   /*!< 0 -> MOL NOT violated, 1 -> MOL violated */
    uint8_t undertemperatureCharge[BS_NR_OF_STRINGS];     /*!< 0 -> MOL NOT violated, 1 -> MOL violated */
    uint8_t undertemperatureDischarge[BS_NR_OF_STRINGS];  /*!< 0 -> MOL NOT violated, 1 -> MOL violated */
    uint8_t cellChargeOvercurrent[BS_NR_OF_STRINGS];      /*!< 0 -> MOL NOT violated, 1 -> MOL violated */
    uint8_t stringChargeOvercurrent[BS_NR_OF_STRINGS];    /*!< 0 -> MOL NOT violated, 1 -> MOL violated */
    uint8_t cellDischargeOvercurrent[BS_NR_OF_STRINGS];   /*!< 0 -> MOL NOT violated, 1 -> MOL violated */
    uint8_t stringDischargeOvercurrent[BS_NR_OF_STRINGS]; /*!< 0 -> MOL NOT violated, 1 -> MOL violated */
    uint8_t pcbOvertemperature[BS_NR_OF_STRINGS];         /*!< 0 -> MOL NOT violated, 1 -> MOL violated */
    uint8_t pcbUndertemperature[BS_NR_OF_STRINGS];        /*!< 0 -> MOL NOT violated, 1 -> MOL violated */
} DATA_BLOCK_MOL_FLAG_s;

/** data block struct of sox */
typedef struct {
    /* This struct needs to be at the beginning of every database entry. During
     * the initialization of a database struct, uniqueId must be set to the
     * respective database entry representation in enum DATA_BLOCK_ID_e. */
    DATA_BLOCK_HEADER_s header;               /*!< Data block header */
    float averageSoc_perc[BS_NR_OF_STRINGS];  /*!< 0.0 <= averageSoc <= 100.0 */
    float minimumSoc_perc[BS_NR_OF_STRINGS];  /*!< 0.0 <= minSoc <= 100.0 */
    float maximumSoc_perc[BS_NR_OF_STRINGS];  /*!< 0.0 <= maxSoc <= 100.0 */
    float averageSoe_perc[BS_NR_OF_STRINGS];  /*!< 0.0 <= averageSoe <= 100.0 */
    float minimumSoe_perc[BS_NR_OF_STRINGS];  /*!< 0.0 <= minimumSoe <= 100.0  */
    float maximumSoe_perc[BS_NR_OF_STRINGS];  /*!< 0.0 <= maximumSoe <= 100.0  */
    float averageSoh_perc[BS_NR_OF_STRINGS];  /*!< 0.0 <= averageSoh <= 100.0 */
    float minimumSoh_perc[BS_NR_OF_STRINGS];  /*!< 0.0 <= minimumSoh <= 100.0  */
    float maximumSoh_perc[BS_NR_OF_STRINGS];  /*!< 0.0 <= maximumSoh <= 100.0  */
    uint32_t maximumSoe_Wh[BS_NR_OF_STRINGS]; /*!< maximum string energy in Wh */
    uint32_t averageSoe_Wh[BS_NR_OF_STRINGS]; /*!< average string energy in Wh */
    uint32_t minimumSoe_Wh[BS_NR_OF_STRINGS]; /*!< minimum string energy in Wh */
} DATA_BLOCK_SOX_s;

/** data block struct of can state request */
typedef struct {
    /* This struct needs to be at the beginning of every database entry. During
     * the initialization of a database struct, uniqueId must be set to the
     * respective database entry representation in enum DATA_BLOCK_ID_e. */
    DATA_BLOCK_HEADER_s header;         /*!< Data block header */
    uint8_t stateRequestViaCan;         /*!< state request */
    uint8_t previousStateRequestViaCan; /*!< previous state request */
    uint8_t stateRequestViaCanPending;  /*!< pending state request */
    uint8_t stateCounter;               /*!< counts state updates */
} DATA_BLOCK_STATEREQUEST_s;

/** data block struct of the moving average algorithm */
typedef struct {
    /* This struct needs to be at the beginning of every database entry. During
     * the initialization of a database struct, uniqueId must be set to the
     * respective database entry representation in enum DATA_BLOCK_ID_e. */
    DATA_BLOCK_HEADER_s header;                        /*!< Data block header */
    float movingAverageCurrent1sInterval_mA;           /*!< current moving average over the last 1s */
    float movingAverageCurrent5sInterval_mA;           /*!< current moving average over the last 5s */
    float movingAverageCurrent10sInterval_mA;          /*!< current moving average over the last 10s */
    float movingAverageCurrent30sInterval_mA;          /*!< current moving average over the last 30s */
    float movingAverageCurrent60sInterval_mA;          /*!< current moving average over the last 60s */
    float movingAverageCurrentConfigurableInterval_mA; /*!< current moving average over the last configured time */
    float movingAveragePower1sInterval_mA;             /*!< power moving average over the last 1s */
    float movingAveragePower5sInterval_mA;             /*!< power moving average over the last 5s */
    float movingAveragePower10sInterval_mA;            /*!< power moving average over the last 10s */
    float movingAveragePower30sInterval_mA;            /*!< power moving average over the last 30s */
    float movingAveragePower60sInterval_mA;            /*!< power moving average over the last 60s */
    float movingAveragePowerConfigurableInterval_mA;   /*!< power moving average over the last configured time */
} DATA_BLOCK_MOVING_AVERAGE_s;

/** data block struct of insulation monitoring device measurement */
typedef struct {
    /* This struct needs to be at the beginning of every database entry. During
     * the initialization of a database struct, uniqueId must be set to the
     * respective database entry representation in enum DATA_BLOCK_ID_e. */
    DATA_BLOCK_HEADER_s header;         /*!< Data block header */
    bool isImdRunning;                  /*!< true -> Insulation resistance measurement active, false -> not active */
    bool isInsulationMeasurementValid;  /*!< true -> resistance value valid, false -> resistance unreliable */
    uint32_t insulationResistance_kOhm; /*!< insulation resistance measured in kOhm */
    bool
        areDeviceFlagsValid; /*!< true -> flags below this database entry valid, false -> flags unreliable e.g. if device error detected */
    bool
        dfIsCriticalResistanceDetected; /*!< device status flag: false -> resistance value okay, true -> resistance value too low/error */
    bool dfIsWarnableResistanceDetected; /*!< true: warning threshold violated, false: no warning active */
    bool dfIsChassisFaultDetected;       /*!< true: short between HV potential and chassis detected, false: no error */
    bool dfIsChassisShortToHvPlus;       /*!< true: bias/tendency to the location of the insulation fault to HV plus */
    bool dfIsChassisShortToHvMinus;      /*!< true: bias/tendency to the location of the insulation fault to HV minus */
    bool dfIsDeviceErrorDetected;        /*!< true: device error detected, false: no error detected */
    bool dfIsMeasurmentedUpToDate;       /*!< true: measurement up to-date, false: outdated */
} DATA_BLOCK_INSULATION_MONITORING_s;

/** data block struct for the I2C humidity/temperature sensor */
typedef struct {
    /* This struct needs to be at the beginning of every database entry. During
     * the initialization of a database struct, uniqueId must be set to the
     * respective database entry representation in enum DATA_BLOCK_ID_e. */
    DATA_BLOCK_HEADER_s header; /*!< Data block header */
    int16_t temperature_ddegC;
    uint8_t humidity_perc;
} DATA_BLOCK_HTSEN_s;

/** data block struct of internal ADC voltage measurement */
typedef struct {
    /* This struct needs to be at the beginning of every database entry. During
     * the initialization of a database struct, uniqueId must be set to the
     * respective database entry representation in enum DATA_BLOCK_ID_e. */
    DATA_BLOCK_HEADER_s header;                               /*!< Data block header */
    float adc1ConvertedVoltages_mV[MCU_ADC1_MAX_NR_CHANNELS]; /*!< voltages measured by the internal ADC ADC1 */
} DATA_BLOCK_ADC_VOLTAGE_s;

/** data block struct for the database built-in self-test */
typedef struct {
    /* This struct needs to be at the beginning of every database entry. During
     * the initialization of a database struct, uniqueId must be set to the
     * respective database entry representation in enum DATA_BLOCK_ID_e. */
    DATA_BLOCK_HEADER_s header; /*!< Data block header */
    uint8_t member1;            /*!< first member of self-test struct */
    uint8_t member2;            /*!< second member of self-test struct */
} DATA_BLOCK_DUMMY_FOR_SELF_TEST_s;

/** array for the database */
typedef struct {
    /* This struct needs to be at the beginning of every database entry. During
     * the initialization of a database struct, uniqueId must be set to the
     * respective database entry representation in enum DATA_BLOCK_ID_e. */
    DATA_BLOCK_HEADER_s header;          /*!< Data block header */
    uint16_t RegenCurrent[5];            /*!< Save the data in waiting to know if its regenerative or charging phase*/
    uint16_t TemperatureRegenCurrent[5]; /*!< Save the data in waiting to know if its regenerative or charging phase*/
    uint16_t RegenerativeTime;           /*!< Record the time during the regenerative time*/
    uint16_t SleepTime;                  /*!< Record the time during the sleep time*/
    uint16_t Ahregen;                    /*!< Save the data in waiting to know if its regenerative or charging phase*/
    uint16_t Whregen;                    /*!< Save the data in waiting to know if its regenerative or charging phase*/
    uint16_t Start_DOC;                  /*!< Start the deep of charge*/
    uint16_t Start_DOD;                  /*!< Start the deep of discharge*/
    uint16_t RegenerativeTable[5][2];    /*!< Save the data in waiting to know if its regenerative or charging phase*/
    bool ChargePhase;                    /*!< true if it is the charge phase*/
    bool DischargePhase;                 /*!< true if it is the charge phase*/
    bool RegenPhase;                     /*!< true if it is the charge phase*/
} DATA_BLOCK_DATABASE_s;

/** array for the database */
typedef struct {
    /* This struct needs to be at the beginning of every database entry. During
     * the initialization of a database struct, uniqueId must be set to the
     * respective database entry representation in enum DATA_BLOCK_ID_e. */
    DATA_BLOCK_HEADER_s header; /*!< Data block header */
    float SigmaX[3][3];         /*!< Uncertainty of initial state (xhat) */
    float SigmaV;               /*!< Uncertainty of Voltage sensor, state equation */
    float SigmaW;               /*!< Uncertainty of current sensor, state equation */
    float xhat[3];              /*!< Estimated state (�rc1, �rc2, S�C) */
    float Ahat[3][3];           /*!< Jacobian matrix of the partial derivatives */
    float Bhat[3];              /*!< Input matrix which represent the offset of the model */
    float Chat[3];              /*!< Partial derivative of the voltage in function of SOC */
    float Dhat;                 /*!< Influence of the internal resistor on the correction factor L */
} DATA_BLOCK_EKF_s;

typedef struct {
    /* This struct needs to be at the beginning of every database entry. During
     * the initialization of a database struct, uniqueId must be set to the
     * respective database entry representation in enum DATA_BLOCK_ID_e. */
    DATA_BLOCK_HEADER_s header; /*!< Data block header */
    float OCV;                  /*!< cell voltage in mV */
    float R0;                   /*!< R0 in [mOhms] */
    float R1;                   /*!< R1 in [mOhms] */
    float R2;                   /*!< R2 in [mOhms] */
    float Tau1;                 /*!< Tau1 in [ms] */
    float Tau2;                 /*!< Tau2 in [ms] */
    float Uest;                 /*!< Cell voltage estimated */
    float U_hyst;               /*!< hysteresis voltage */
    float U_hyst_new;           /*!< new hysteresis voltage value (LFP) */
    float U_hyst_old;           /*!< old hysteresis voltage value (LFP) */
    int time_hyst;              /*!< Time since current is constant (LFP) */
    float SOC_ref;              /*!< SOC reference used for the SOH estimation */
    bool SOC_ref_valid;         /*!< Validation of the actual value is a reference */
    float OCV_old;              /*!< Previous value of the OCV*/
    float SOC_int;              /*!< SOC value from the current integration*/
    float SOC_int_old;          /*!< Previous SOC value from the current integration*/
    float SOC_EKF;              /*!< SOC value estimated by the EKF*/
    float SOC_EKF_old;          /*!< Previous SOC value from the EKF*/
    float last_SOC_ECM_check;   /*!< The value of the SOC when the ECM parameters were update*/
    float SOH;                  /*!< SOH state of the battery*/
    float SOH_old;              /*!< Previous SOH value*/
    bool chemistry;             /*!< NMC =>0 | LFP =>1*/
    bool SOC_validate_LFP;      /*!< The SOC of the LFP hs been validated*/
    int temperature_set;        /*!< temperature set for the ECM [0, 10, 25, 45]*/
    int16_t Current_mA_old;     /*!< Previous Current measurement*/
} DATA_BLOCK_ECM_s;

typedef struct {
    /* This struct needs to be at the beginning of every database entry. During
     * the initialization of a database struct, uniqueId must be set to the
     * respective database entry representation in enum DATA_BLOCK_ID_e. */
    DATA_BLOCK_HEADER_s header; /*!< Data block header */
    //uint16_t T_HR0;             Total seconds in Mode 0 (deep sleep)
    //uint16_t T_HR1;             Total seconds in Mode 1 (sleep)
    //uint16_t T_HR2;             Total seconds in Mode 2 (idle)
    //uint16_t T_HR3;             Total seconds in Mode 3 (charge)
    //uint16_t T_HR4;             Total seconds in Mode 4 (discharge)
    //uint16_t T_HR5;             Total seconds in Mode 5 (regen)
    uint16_t HR[6];
} DATA_BLOCK_RD_BMS_OP_TIME_s;

typedef struct {
    /* This struct needs to be at the beginning of every database entry. During
     * the initialization of a database struct, uniqueId must be set to the
     * respective database entry representation in enum DATA_BLOCK_ID_e. */
    DATA_BLOCK_HEADER_s header; /*!< Data block header */
    //uint16_t T_WHC;             Total Watthour Charged
    //uint16_t T_WHD;             Total Watthour discharged
    //uint16_t T_WHR;             Total Watthour charged in regen
    //uint16_t T_AHC;             Total Amphour charged
    //uint16_t T_AHD;             Total Amphour discharged
    //uint16_t T_AHR;             Total Amphour charged in regen
    uint16_t CONSUMPTION[6];
} DATA_BLOCK_RD_BMS_CONSUMPTION_s;
typedef struct {
    /* This struct needs to be at the beginning of every database entry. During
     * the initialization of a database struct, uniqueId must be set to the
     * respective database entry representation in enum DATA_BLOCK_ID_e. */
    DATA_BLOCK_HEADER_s header; /*!< Data block header */
    //uint16_t EOC100;            Number of times End of charge between 95-100%
    //uint16_t EOC94;             Number of times End of charge between 90-94%
    //uint16_t EOC89;             Number of times End of charge between 85-89%
    //uint16_t EOC84;             Number of times End of charge between 80-84%
    //uint16_t EOC79;             Number of times End of charge between 60-79%
    //uint16_t EOC59;             Number of times End of charge between 40-59%
    //uint16_t EOC39;             Number of times End of charge between 20-39%
    //uint16_t EOC19;             Number of times End of charge between 0-19%
    uint16_t EOC[8];
} DATA_BLOCK_RD_BMS_EOC_s;
typedef struct {
    /* This struct needs to be at the beginning of every database entry. During
     * the initialization of a database struct, uniqueId must be set to the
     * respective database entry representation in enum DATA_BLOCK_ID_e. */
    DATA_BLOCK_HEADER_s header; /*!< Data block header */
    //uint16_t EOD100;            Number of times End of discharge between 87.5-100%
    //uint16_t EOD87;             Number of times End of discharge between 75-87.5%
    //uint16_t EOD75;             Number of times End of discharge between 62.5-75%
    //uint16_t EOD62;             Number of times End of discharge between 50-62.5%
    //uint16_t EOD50;             Number of times End of discharge between 37.5-50%
    //uint16_t EOD37;             Number of times End of discharge between 25-37.5%
    //uint16_t EOD25;             Number of times End of discharge between 12.5-25%
    //uint16_t EOD12;             Number of times End of discharge between 0-12.5%
    uint16_t EOD[8];
} DATA_BLOCK_RD_BMS_EOD_s;
typedef struct {
    /* This struct needs to be at the beginning of every database entry. During
     * the initialization of a database struct, uniqueId must be set to the
     * respective database entry representation in enum DATA_BLOCK_ID_e. */
    DATA_BLOCK_HEADER_s header; /*!< Data block header */
    //uint16_t DOD100;            Depth of Discharge 87.5- 100%
    //uint16_t DOD87;             Depth of Discharge 75- 87.5%
    //uint16_t DOD75;             Depth of Discharge 62.5- 75%
    //uint16_t DOD62;             Depth of Discharge 50- 62.5%
    //uint16_t DOD50;             Depth of Discharge 37.5- 50%
    //uint16_t DOD37;             Depth of Discharge 25- 37.5%
    //uint16_t DOD25;             Depth of Discharge 12.5- 25%
    //uint16_t DOD12;             Depth of Discharge 0- 12.5%
    uint16_t DOD[8];
} DATA_BLOCK_RD_BMS_DOD_s;
typedef struct {
    /* This struct needs to be at the beginning of every database entry. During
     * the initialization of a database struct, uniqueId must be set to the
     * respective database entry representation in enum DATA_BLOCK_ID_e. */
    DATA_BLOCK_HEADER_s header; /*!< Data block header */
    //uint16_t DOC100;            Depth of Charge 87.5- 100%
    //uint16_t DOC87;             Depth of Charge 75- 87.5%
    //uint16_t DOC75;             Depth of Charge 62.5- 75%
    //uint16_t DOC62;             Depth of Charge 50- 62.5%
    //uint16_t DOC50;             Depth of Charge 37.5- 50%
    //uint16_t DOC37;             Depth of Charge 25- 37.5%
    //uint16_t DOC25;             Depth of Charge 12.5- 25%
    //uint16_t DOC12;             Depth of Charge 0- 12.5%
    uint16_t DOC[8];
} DATA_BLOCK_RD_BMS_DOC_s;
typedef struct {
    /* This struct needs to be at the beginning of every database entry. During
     * the initialization of a database struct, uniqueId must be set to the
     * respective database entry representation in enum DATA_BLOCK_ID_e. */
    DATA_BLOCK_HEADER_s header; /*!< Data block header */
    //uint16_t DCC1;              Time spent at Discharge Current Level 1 (0-20%)
    //uint16_t DCC2;              Time spent at Discharge Current Level 2 (20-40%)
    //uint16_t DCC3;              Time spent at Discharge Current Level 3 (40-60%)
    //uint16_t DCC4;              Time spent at Discharge Current Level 4 (60-80%)
    //uint16_t DCC5;              Time spent at Discharge Current Level 5 (80-100%)
    uint16_t DCC[5];
} DATA_BLOCK_RD_BMS_CUMTIME_DISCHARGE_s;
typedef struct {
    /* This struct needs to be at the beginning of every database entry. During
     * the initialization of a database struct, uniqueId must be set to the
     * respective database entry representation in enum DATA_BLOCK_ID_e. */
    DATA_BLOCK_HEADER_s header; /*!< Data block header */
    //uint16_t CCC1;              Time spent at Charge Current Level 1 (0-20%)
    //uint16_t CCC2;              Time spent at Charge Current Level 2 (20-40%)
    //uint16_t CCC3;              Time spent at Charge Current Level 3 (40-60%)
    //uint16_t CCC4;              Time spent at Charge Current Level 4 (60-80%)
    //uint16_t CCC5;              Time spent at Charge Current Level 5 (80-100%)
    uint16_t CCC[5];
} DATA_BLOCK_RD_BMS_CUMTIME_CHARGE_s;
typedef struct {
    /* This struct needs to be at the beginning of every database entry. During
     * the initialization of a database struct, uniqueId must be set to the
     * respective database entry representation in enum DATA_BLOCK_ID_e. */
    DATA_BLOCK_HEADER_s header; /*!< Data block header */
    //uint16_t REG1;              Time spent at Regen Current Level 1 (0-20%)
    //uint16_t REG2;              Time spent at Regen Current Level 2 (20-40%)
    //uint16_t REG3;              Time spent at Regen Current Level 3 (40-60%)
    //uint16_t REG4;              Time spent at Regen Current Level 4 (60-80%)
    //uint16_t REG5;              Time spent at Regen Current Level 5 (80-100%)
    uint16_t REG[5];
} DATA_BLOCK_RD_BMS_CUMTIME_REGEN_s;
typedef struct {
    /* This struct needs to be at the beginning of every database entry. During
     * the initialization of a database struct, uniqueId must be set to the
     * respective database entry representation in enum DATA_BLOCK_ID_e. */
    DATA_BLOCK_HEADER_s header; /*!< Data block header */
    //uint16_t OPT1;              Time spent at Operation Temp Level 1 (0-20%)
    //uint16_t OPT2;              Time spent at Operation Temp Level 2 (20-40%)
    //uint16_t OPT3;              Time spent at Operation Temp Level 3 (40-60%)
    //uint16_t OPT4;              Time spent at Operation Temp Level 4 (60-80%)
    //uint16_t OPT5;              Time spent at Operation Temp Level 5 (80-100%)
    uint16_t OPT[5];
} DATA_BLOCK_RD_BMS_CUMTIME_OPTEMP_s;
typedef struct {
    /* This struct needs to be at the beginning of every database entry. During
     * the initialization of a database struct, uniqueId must be set to the
     * respective database entry representation in enum DATA_BLOCK_ID_e. */
    DATA_BLOCK_HEADER_s header; /*!< Data block header */
    //uint16_t CPT1;              Time spent at Charge Temp Level 1 (0-20%)
    //uint16_t CPT2;              Time spent at Charge Temp Level 2 (20-40%)
    //uint16_t CPT3;              Time spent at Charge Temp Level 3 (40-60%)
    //uint16_t CPT4;              Time spent at Charge Temp Level 4 (60-80%)
    //uint16_t CPT5;              Time spent at Charge Temp Level 5 (80-100%)
    uint16_t CPT[5];
} DATA_BLOCK_RD_BMS_CUMTIME_CHARGE_TEMP_s;
typedef struct {
    /* This struct needs to be at the beginning of every database entry. During
     * the initialization of a database struct, uniqueId must be set to the
     * respective database entry representation in enum DATA_BLOCK_ID_e. */
    DATA_BLOCK_HEADER_s header; /*!< Data block header */
    //uint16_t DPT1;              Time spent at Discharge Temp Level 1 (0-20%)
    //uint16_t DPT2;              Time spent at Discharge Temp Level 2 (20-40%)
    //uint16_t DPT3;              Time spent at Discharge Temp Level 3 (40-60%)
    //uint16_t DPT4;              Time spent at Discharge Temp Level 4 (60-80%)
    //uint16_t DPT5;              Time spent at Discharge Temp Level 5 (80-100%)
    uint16_t DPT[5];
} DATA_BLOCK_RD_BMS_CUMTIME_DISCHARGE_TEMP_s;
typedef struct {
    /* This struct needs to be at the beginning of every database entry. During
     * the initialization of a database struct, uniqueId must be set to the
     * respective database entry representation in enum DATA_BLOCK_e. */
    DATA_BLOCK_HEADER_s header; /*!< Data block header */
    //uint16_t REGT1;             Time spent at Regen Temp Level 1 (0-20%)
    //uint16_t REGT2;             Time spent at Regen Temp Level 2 (20-40%)
    //uint16_t REGT3;             Time spent at Regen Temp Level 3 (40-60%)
    //uint16_t REGT4;             Time spent at Regen Temp Level 4 (60-80%)
    //uint16_t REGT5;             Time spent at Regen Temp Level 5 (80-100%)
    uint16_t REGT[5];
} DATA_BLOCK_RD_BMS_CUMTIME_REGEN_TEMP_s;
typedef struct {
    /* This struct needs to be at the beginning of every database entry. During
     * the initialization of a database struct, uniqueId must be set to the
     * respective database entry representation in enum DATA_BLOCK_e. */
    DATA_BLOCK_HEADER_s header; /*!< Data block header */
    //uint16_t SOC12;             Time spent at SoC Level 1 (0-12.5%)
    //uint16_t SOC25;             Time spent at SoC Level 2 (12.5-25%)
    //uint16_t SOC37;             Time spent at SoC Level 3 (25-37.5%)
    //uint16_t SOC50;             Time spent at SoC Level 4 (37.5-50%)
    //uint16_t SOC62;             Time spent at SoC Level 5 (50-62.5%)
    //uint16_t SOC75;             Time spent at SoC Level 6 (62.5-75%)
    //uint16_t SOC87;             Time spent at SoC Level 7 (75-87.5%)
    //uint16_t SOC100;            Time spent at SoC Level 8 (87.5-100%)
    uint16_t SOC[8];
} DATA_BLOCK_RD_BMS_CUMTIME_SOC_s;
extern DATA_BASE_s data_database[DATA_BLOCK_ID_MAX];

/*========== Extern Constant and Variable Declarations ======================*/

/*========== Extern Function Prototypes =====================================*/

/*========== Externalized Static Functions Prototypes (Unit Test) ===========*/

#endif /* FOXBMS__DATABASE_CFG_H_ */
