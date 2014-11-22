#ifndef DEFS_h
#define DEFS_h

// Version of the EEPROM parameter table
#define EEPROM_FORMAT_VS     120

#if CONFIG_HAL_BOARD == HAL_BOARD_APM1
  #define A_LED_PIN          37
  #define B_LED_PIN          36
  #define C_LED_PIN          35
  #define LED_ON             HIGH
  #define LED_OFF            LOW
  #define SLIDE_SWITCH_PIN   40
  #define PUSHBUTTON_PIN     41
  #define USB_MUX_PIN        -1
  #define CLI_SLIDER_ENABLED DISABLED
  #define BATTERY_VOLT_PIN   0      // Battery voltage on A0
  #define BATTERY_CURR_PIN   1      // Battery current on A1
#elif CONFIG_HAL_BOARD == HAL_BOARD_APM2
  #define A_LED_PIN          27
  #define B_LED_PIN          26
  #define C_LED_PIN          25
  #define LED_ON             LOW
  #define LED_OFF            HIGH
  #define SLIDE_SWITCH_PIN   (-1)
  #define PUSHBUTTON_PIN     (-1)
  #define CLI_SLIDER_ENABLED DISABLED
  #define USB_MUX_PIN        23
  #define BATTERY_VOLT_PIN   1      // Battery voltage on A1
  #define BATTERY_CURR_PIN   2      // Battery current on A2
#endif
//////////////////////////////////////////////////////////////////////////////////////////
// Battery statistics
//////////////////////////////////////////////////////////////////////////////////////////
#define BATT_ATTO_3DR        1

#define BATT_MIN_VOLTAGE     9.0    // 3 cells @ 3.0 V
#define BATT_MAX_VOLTAGE     25.2   // 6 cells @ 3.7 V
#define BATT_T_MS            100
#define BATT_CAP_mAh         10000

//////////////////////////////////////////////////////////////////////////////////////////
// General settings
//////////////////////////////////////////////////////////////////////////////////////////
#define DEBUG_OUT            0
#define BENCH_OUT            0

#define NR_OF_PIDS           10
// PID indices
#define PID_PIT_RATE         0      // From Dr. Owen..
#define PID_ROL_RATE         1
#define PID_PIT_STAB         2
#define PID_ROL_STAB         3
#define PID_YAW_RATE         4
#define PID_YAW_STAB         5
// Optional altitude hold
#define PID_THR_RATE         6      // For my altitude hold implementation
#define PID_THR_STAB         7      // For my altitude hold implementation
#define PID_ACC_RATE         8      // For my altitude hold implementation
#define PID_ACC_STAB         9      // For my altitude hold implementation

// Motor numbers definitions for X configuration
#define MOTOR_FR             0      // Front right  (CW)
#define MOTOR_BL             1      // back left    (CW)
#define MOTOR_FL             2      // Front left   (CCW)
#define MOTOR_BR             3      // back right   (CCW)

// Radio min/max values for each stick for my radio (worked out at beginning of article)
#define RC_THR_OFF           1000   // Motors completely off
#define RC_THR_ACRO          1225   // Minimum throttle to begin with stabilization
#define RC_THR_MAX           2000   // Maximum throttle bias
#define RC_THR_80P           1675   // Maximum allowed throttle value, settable by user

// Degree range for remote control
#define RC_YAW_MIN           -180
#define RC_YAW_MAX           +180
#define RC_PIT_MIN           -45
#define RC_PIT_MAX           +45
#define RC_ROL_MIN           -45
#define RC_ROL_MAX           +45

//////////////////////////////////////////////////////////////////////////////////////////
// Setup serial ports
//////////////////////////////////////////////////////////////////////////////////////////
#define BAUD_RATE_A          115200 // IO USB
#define BAUD_RATE_B          38400  // GPS
#define BAUD_RATE_C          57600  // 3DR RADIO

//////////////////////////////////////////////////////////////////////////////////////////
// Main loop
//////////////////////////////////////////////////////////////////////////////////////////
#define RCVR_T_MS            20     // Update frequency of the receiver loop: 50 Hz
#define INAV_T_MS            20     // Update frequency for the auto navigation system: 50 Hz
#define COMP_T_MS            25000  // Save compass offsets

//////////////////////////////////////////////////////////////////////////////////////////
// Scheduler module
//////////////////////////////////////////////////////////////////////////////////////////
#define NO_PRC_SCHED         16     // Maximum number of processes in scheduler

//////////////////////////////////////////////////////////////////////////////////////////
// Receiver module
//////////////////////////////////////////////////////////////////////////////////////////
#define IN_BUFFER_S          256

#define RC_ROL               0
#define RC_PIT               1
#define RC_THR               2
#define RC_YAW               3

#define RADIO_MSG_LENGTH     7        // Maximum length of radio control message without stop byte
#define APM_IOCHAN_CNT 	     8

#define UART_A               0
#define UART_B               1
#define UART_C               2

#define USE_RCIN             0        // This firmware is not using any PPM radio as default
#define USE_UART_A           1        // This is the standard input source (USB from Raspberry Pi)
#define USE_UART_C           1        // And this is the fall-back option (3DR Radio 433 or 900 MHz)

#define COM_PKT_TIMEOUT      500      // Time-out in ms; If the time-out is triggered the machine will go down
#define RCIN_TIMEOUT         250      // Time-out of the ppm radio in ms; If time-out is triggered the firmware tries to receive packets via the USB port on uartA
#define UART_A_TIMEOUT       250      // Time-out of the console serial port in ms; If time-out is triggered the firmware tries to receive packets via the 3DR radio on uartC

#define PID_ARGS             6        // Nr of arguments for PID configuration
#define PID_BUFFER_S         5

#define GYRO_ARGS            2        // Nr. of arguments for on-flight drift compensation
#define COMP_ARGS            5        // Nr. of arguments for compass offset compensation
#define GPSP_ARGS            4        // Nr. of arguments for GPSPosition structure

//////////////////////////////////////////////////////////////////////////////////////////
// Device module
//////////////////////////////////////////////////////////////////////////////////////////
#define INERT_FUSION_RATE    4.5f     // Sensor fusion rate: higher => faster annealing

#define BAROM_LOWPATH_FILT_f 0.35f    // Filter constant for the barometer
#define COMPA_LOWPATH_FILT_f 0.25f    // Filter constant for the compass
#define INERT_LOWPATH_FILT_f 0.15f    // Filter constant for the accelerometer

#define INERT_FFALL_BIAS     75       // Accelerometer bias in cm/s². If z-axis values are less than 0.75 m/s², break annealing to accelerometer for attitude estimation
#define INERT_ANGLE_BIAS     60

#define INERT_G_CONST        9.81f

#define COMPASS_UPDATE_T     100

#define RANGE_FINDER_PIN     13
#define RANGE_FINDER_SCALE   1.0f

//////////////////////////////////////////////////////////////////////////////////////////
// Error handling
//////////////////////////////////////////////////////////////////////////////////////////
#define VOLTAGE_ALARM_LOW    10.f     // 2.5V per LiPo cell is already close to death of the cell
#define VOLTAGE_ALARM_HIGH   21.f     // 5 * 4.2 V (my setup)
#define THR_MOD_STEP_S       1.25f
#define THR_TAKE_OFF         1300
#define THR_MIN_STEP_S       25.f
#define MAX_FALL_SPEED_MS    0.833f

//////////////////////////////////////////////////////////////////////////////////////////
// Auto navigation
//////////////////////////////////////////////////////////////////////////////////////////
#define MAX_YAW              15
#define MAX_PIT              15
#define YAW_ZERO_SLOPE       25.f
#define YAW_ZERO_MOD         5.0f
#define YAW_CTRL_SLOPE       5.0f
#define YAW_CTRL_MOD         1.0f
#define YAW_ERROR_RATE       5.0f

#define HLD_ALTITUDE_TIMER   20     // 50 Hz

#define HLD_ALTITUDE_ZGBIAS  0.25f  // in g
#define HLD_ALTITUDE_ZTBIAS  25     // in ms

#endif /*DEFS_h*/
