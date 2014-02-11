#ifndef BATTMONITOR_H
#define BATTMONITOR_H

#include <AP_BattMonitor.h>


#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
  // Pins for AttoPilot sensor modules for APM 2
  # define AP_ATTO_VOLT_PIN 1
  # define AP_ATTO_CURR_PIN 2
#else
  # define AP_ATTO_VOLT_PIN AP_BATT_VOLT_PIN
  # define AP_ATTO_CURR_PIN AP_BATT_CURR_PIN
#endif

// Devider values
# define AP_BATT_VOLTDIVIDER_ATTO180 15.70   // Volt divider for AttoPilot 50V/180A sensor
# define AP_BATT_VOLTDIVIDER_ATTO90 15.70   // Volt divider for AttoPilot 50V/90A sensor
# define AP_BATT_VOLTDIVIDER_ATTO45 4.127   // Volt divider for AttoPilot 13.6V/45A sensor
# define AP_BATT_CURR_AMP_PERVOLT_ATTO180 54.64  // Amp/Volt for AttoPilot 50V/180A sensor
# define AP_BATT_CURR_AMP_PERVOLT_ATTO90 27.32  // Amp/Volt for AttoPilot 50V/90A sensor
# define AP_BATT_CURR_AMP_PERVOLT_ATTO45 13.66  // Amp/Volt for AttoPilot 13.6V/45A sensor
// battery monitor types
# ifndef AP_BATT_CAPACITY_DEFAULT
  # define AP_BATT_CAPACITY_DEFAULT            10000
# endif


enum BATT_SENSOR_TYPE {
  ATTO45 = 0,
  ATTO90 = 1,
  ATTO180 = 2
    // Maybe extent for other stuff
};

/*
 * AP_Param independent version of the battery monitor
 */
class BattMonitor : 
public AP_BattMonitor {
public: 
  /*
   * Changes source of battery monitor
   * Makes usage of AP_Param unnecessary
   */
  void setup_source( int8_t volt_pin, 
  int8_t curr_pin, 
  float volt_multiplier, 
  float curr_amp_per_volt, 
  int32_t pack_capacity, 
  float curr_amp_offset, 
  int8_t monitoring );
  /*
   * Calls function above
   * If no proper source can be recognized, the standard settings (3DR power module) are used.
   */
  void setup_source(const unsigned int &t = ATTO180);
};

#endif

