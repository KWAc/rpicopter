#ifndef BATTMONITOR_H
#define BATTMONITOR_H

#include <AP_BattMonitor.h>
#include "config.h"

// Pins for AttoPilot sensor modules for APM 2
#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
  # define AP_ATTO_VOLT_PIN 1
  # define AP_ATTO_CURR_PIN 2
#else
  # define AP_ATTO_VOLT_PIN AP_BATT_VOLT_PIN
  # define AP_ATTO_CURR_PIN AP_BATT_CURR_PIN
#endif

// Divider values
# define AP_BATT_VOLTDIVIDER_ATTO180 15.70        // Volt divider for AttoPilot 50V/180A sensor
# define AP_BATT_VOLTDIVIDER_ATTO90 15.70         // Volt divider for AttoPilot 50V/90A sensor
# define AP_BATT_VOLTDIVIDER_ATTO45 4.127         // Volt divider for AttoPilot 13.6V/45A sensor
# define AP_BATT_CURR_AMP_PERVOLT_ATTO180 54.64   // Amp/Volt for AttoPilot 50V/180A sensor
# define AP_BATT_CURR_AMP_PERVOLT_ATTO90 27.32    // Amp/Volt for AttoPilot 50V/90A sensor
# define AP_BATT_CURR_AMP_PERVOLT_ATTO45 13.66    // Amp/Volt for AttoPilot 13.6V/45A sensor


/*
 * AP_Param independent version of the battery monitor
 */
class BattMonitor : public AP_BattMonitor {
public:
  enum BATT_SENSOR_TYPE {
    ATTO45  = 0,
    ATTO90  = 1,
    ATTO180 = 2
  };

  void setup_mon ( const int iType    = AP_BATT_MONITOR_VOLTAGE_AND_CURRENT );
  void setup_type( const int iType    = BattMonitor::ATTO180 );
  void setup_pins( const int iVoltPin = AP_BATT_VOLT_PIN, 
                   const int iCurrPin = AP_BATT_CURR_PIN );
  void setup_cap ( const int iCap_mAh = BATT_CAP_mAh );
};

#endif

