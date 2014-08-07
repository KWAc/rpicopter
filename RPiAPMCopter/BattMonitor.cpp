#include "BattMonitor.h"


void BattMonitor::setup_mon(const int iType) {
  _monitoring = iType;
}

void BattMonitor::setup_cap(const int iCap) {
  _pack_capacity = iCap;
}

void BattMonitor::setup_pins(const int iVolt, const int iCur) {
  _volt_pin = iVolt;
  _curr_pin = iCur;
}

void BattMonitor::setup_type(const int t) {
  switch (t) {
    case ATTO45:
      _volt_multiplier   = AP_BATT_VOLTDIVIDER_ATTO45;
      _curr_amp_per_volt = AP_BATT_CURR_AMP_PERVOLT_ATTO45;
      setup_pins(AP_ATTO_VOLT_PIN, AP_ATTO_CURR_PIN);
      setup_mon(AP_BATT_MONITOR_VOLTAGE_AND_CURRENT);
    break;
    case ATTO90:
      _volt_multiplier   = AP_BATT_VOLTDIVIDER_ATTO90;
      _curr_amp_per_volt = AP_BATT_CURR_AMP_PERVOLT_ATTO90;
      setup_pins(AP_ATTO_VOLT_PIN, AP_ATTO_CURR_PIN);
      setup_mon(AP_BATT_MONITOR_VOLTAGE_AND_CURRENT);
    break;
    case ATTO180:
      _volt_multiplier   = AP_BATT_VOLTDIVIDER_ATTO180;
      _curr_amp_per_volt = AP_BATT_CURR_AMP_PERVOLT_ATTO180;
      setup_pins(AP_ATTO_VOLT_PIN, AP_ATTO_CURR_PIN);
      setup_mon(AP_BATT_MONITOR_VOLTAGE_AND_CURRENT);
    break;
    default:
      _volt_multiplier   = AP_BATT_VOLTDIVIDER_DEFAULT;
      _curr_amp_per_volt = AP_BATT_CURR_AMP_PERVOLT_DEFAULT;
      setup_pins(AP_BATT_VOLT_PIN, AP_BATT_CURR_PIN);
      setup_mon(AP_BATT_MONITOR_DISABLED);
    break;
  }
}
