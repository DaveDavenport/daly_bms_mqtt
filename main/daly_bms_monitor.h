#ifndef DALY_BMS_MONITOR_H
#define DALY_BMS_MONITOR_H
#include <stdint.h>

/**
 * The different commands we can send.
 * These are taken from the manufacturer pdf.
 */
typedef enum DalyCommandID {
  // Total SOC, Voltage and current.
  CMD_ID_SOC_TOTAL_VOLTAGE = 0x90,
  // Minimum and maximum cell voltage and matching cell id.
  CMD_ID_MIN_MAX_CELL_VOLTAGE = 0x91,
  // Minimum and maximum temperature and matching cell id.
  CMD_ID_MIN_MAX_TEMPERATURE = 0x92,
  //
  CMD_ID_CHARGE_STATE_REM_CAP = 0x93,
  //
  CMD_ID_STATUS_INFO_1 = 0x94,
  // Voltages of each cell.
  CMD_ID_CELL_VOLTAGES = 0x95,
  // Temperature of each sensor.
  CMD_ID_TEMPERATURES = 0x96,
  //
  CMD_ID_CELL_EQUILIBRIUM_STATE = 0x97,
  //
  CMD_ID_BATTERY_FAILURE_STATE = 0x98
} DalyCommandID;

/**
 * Index into header.
 */
typedef enum DalyMsgIndex {
  CMD_INDEX_START = 0,
  CMD_INDEX_ADDRESS = 1,
  CMD_INDEX_DATA_ID = 2,
  CMD_INDEX_DATA_LEN = 3,
  CMD_INDEX_DATA = 4,
} DalyMsgIndex;

/**
 * Structures for each of the message type.
 */

/**
 * Structure for: CMD_ID_SOC_TOTAL_VOLTAGE
 */
typedef struct {
  // Voltage (V)
  float pressure;
  // Unsure
  float aquisition;
  // Current (A)
  float current;
  // State of charge (%)
  float soc;
} SocTotalVoltage;

/**
 * Structure for: CMD_ID_MIN_MAX_CELL_VOLTAGE
 */
typedef struct {
  uint16_t max_mv;
  uint8_t max_id;
  uint16_t min_mv;
  uint8_t min_id;
} MinMaxCellVoltage;

/**
 * Structure for: CMD_ID_MIN_MAX_TEMPERATURE
 */
typedef struct {
  int16_t max_temp;
  uint8_t max_id;
  int16_t min_temp;
  uint8_t min_id;
} MinMaxCellTemperature;

/**
 * Structure for: CMD_ID_CELL_VOLTAGES
 */
typedef struct {
  uint8_t frame_num;
  uint16_t mvoltage[3];
} CellVoltages;

/**
 * Structure for: CMD_ID_TEMPERATURES
 */
typedef struct {
  uint8_t frame_num;
  uint16_t temp[3];
} Temperatures;

/**
 * Structure for: CMD_ID_CHARGE_STATE_REM_CAP
 */
typedef struct {
  // charge/discharge (0 - stationary, 1 - charged, 2 - discharged )
  uint8_t status;
  // charge status
  uint8_t charge;
  // discharge status
  uint8_t discharge;
  //  bms cycle
  uint8_t life;
  // residual capacity.
  uint32_t residual_charge;
} ChargeState;

/**
 * Structure for: CMD_ID_STATUS_INFO_1
 */
typedef struct {
  // battery string?
  uint8_t battery;
  // temperature
  uint8_t temperature;
  // charger status
  uint8_t charger;
  // load status
  uint8_t load;
  // cycle count.
  uint16_t cycles;

} StatusInfo1;
/**
 * A parent structure for daly message.
 * Usefull for passing messages via a queue.
 */
typedef struct {
  // Type of message this structure hold.
  DalyCommandID id;
  union {
    SocTotalVoltage soc;
    MinMaxCellVoltage mmcv;
    MinMaxCellTemperature mmt;
    CellVoltages cvs;
    Temperatures tmps;
    ChargeState cs;
    StatusInfo1 status;
  };
} DalyMsg;
#endif // DALY_BMS_MONITOR_H
