#ifndef ABS_ERR_h
#define ABS_ERR_h


class AbsErrorDevice {
public:
  enum DEVICE_ERROR_FLAGS {
    NOTHING_F         = 1 << 0, // 1
    GYROMETER_F       = 1 << 1, // 2
    ACCELEROMETR_F    = 1 << 2, // 4
    BAROMETER_F       = 1 << 3, // 8
    COMPASS_F         = 1 << 4, // 16
    GPS_F             = 1 << 5, // 32
    VOLTAGE_HIGH_F    = 1 << 6, // 64
    VOLTAGE_LOW_F     = 1 << 7, // 128
    CURRENT_HIGH_F    = 1 << 8, // 256
    CURRENT_LOW_F     = 1 << 9, // 512
    UART_TIMEOUT_F    = 1 << 10 // 1024
  };

protected:
  // Flag holding device errors
  DEVICE_ERROR_FLAGS m_eErrors;   // Flag will be set in this class if problem occurs, but handling and releasing is part of class "exceptions"

public:
  AbsErrorDevice();
  
  DEVICE_ERROR_FLAGS get_errors();
  void set_errors(DEVICE_ERROR_FLAGS flags);
};

#endif
