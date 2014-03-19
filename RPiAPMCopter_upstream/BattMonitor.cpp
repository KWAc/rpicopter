#include "BattMonitor.h"


void BattMonitor::setup_source(int_fast8_t volt_pin,
                        int_fast8_t curr_pin,
                        float volt_multiplier,
                        float curr_amp_per_volt,
                        int_fast32_t pack_capacity,
                        float curr_amp_offset,
                        int_fast8_t monitoring )
{
  _monitoring         = monitoring;
  _volt_pin           = volt_pin;
  _curr_pin           = curr_pin;
  _volt_multiplier    = volt_multiplier;
  _curr_amp_per_volt  = curr_amp_per_volt;
  _curr_amp_offset    = curr_amp_offset;
  _pack_capacity      = pack_capacity;

  init();
}

void BattMonitor::setup_source(const int &t) {
  switch (t) {
    case ATTO45:
      setup_source(AP_ATTO_VOLT_PIN, AP_ATTO_CURR_PIN, AP_BATT_VOLTDIVIDER_ATTO45, AP_BATT_CURR_AMP_PERVOLT_ATTO45, AP_BATT_CAPACITY_DEFAULT, 0, AP_BATT_MONITOR_VOLTAGE_AND_CURRENT);
    break;
    case ATTO90:
      setup_source(AP_ATTO_VOLT_PIN, AP_ATTO_CURR_PIN, AP_BATT_VOLTDIVIDER_ATTO90, AP_BATT_CURR_AMP_PERVOLT_ATTO90, AP_BATT_CAPACITY_DEFAULT, 0, AP_BATT_MONITOR_VOLTAGE_AND_CURRENT);
    break;
    case ATTO180:
      setup_source(AP_ATTO_VOLT_PIN, AP_ATTO_CURR_PIN, AP_BATT_VOLTDIVIDER_ATTO180, AP_BATT_CURR_AMP_PERVOLT_ATTO180, AP_BATT_CAPACITY_DEFAULT, 0, AP_BATT_MONITOR_VOLTAGE_AND_CURRENT);
    break;
    default:
      setup_source(AP_BATT_VOLT_PIN, AP_BATT_CURR_PIN, AP_BATT_VOLTDIVIDER_DEFAULT, AP_BATT_CURR_AMP_PERVOLT_DEFAULT, AP_BATT_CAPACITY_DEFAULT, 0, AP_BATT_MONITOR_VOLTAGE_AND_CURRENT);
    break;
  }
}
