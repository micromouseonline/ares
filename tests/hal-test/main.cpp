#include <iostream>

/// this would go in a "board-config.h" file

// pick your board
// #define BOARD_A
#define BOARD_B

// Board A configuration
#ifdef BOARD_A
#define BOARD_NAME "DECIMUS"
#define WALLSENSOR_PRESENT
#define IR_SENSOR

#define IMU_PRESENT
#define DIGITAL_IMU
#endif

// Board B configuration
#ifdef BOARD_B
#define BOARD_NAME "MR32"
#define WALLSENSOR_PRESENT
#define TOF_SENSOR

#define IMU_PRESENT
#define ANALOGUE_IMU
#endif
/////////////////////////////////////////////////

#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

class WallSensorHAL {
 public:
  virtual void initialize() = 0;
  virtual float getDistance() = 0;
  virtual ~WallSensorHAL() = default;
};

class ReflectiveIRSensor : public WallSensorHAL {
 public:
  void initialize() override {
    std::cout << "Initializing reflective IR sensor" << std::endl;
  }
  float getDistance() override {
    std::cout << "Getting distance from reflective IR sensor" << std::endl;
    return 89.0f;
  }
};

class TOFSensor : public WallSensorHAL {
 public:
  void initialize() override {
    std::cout << "Initializing TOF sensor" << std::endl;
  }
  float getDistance() override {
    std::cout << "Getting distance from TOF sensor" << std::endl;
    return 67.0f;
  }
};

/////////////////////////////////////////////////

class IMUHAL {
 public:
  virtual void initialize() = 0;
  virtual float getYawRate() = 0;
  virtual ~IMUHAL() = default;
};

class DigitalIMU : public IMUHAL {
 public:
  void initialize() override {
    std::cout << "Initializing digital IMU" << std::endl;
  }
  float getYawRate() override {
    std::cout << "Getting yaw rate from digital IMU" << std::endl;
    return 234.0f;
  }
  void calibrate() {
    std::cout << "calibrating the digital IMU" << std::endl;
  }
};

class AnalogIMU : public IMUHAL {
 public:
  void initialize() override {
    std::cout << "Initializing analog IMU" << std::endl;
  }
  float getYawRate() override {
    std::cout << "Getting yaw rate from analog IMU" << std::endl;
    return 123.0f;
  }
};

////////////////////////////////////////////////

class Board {
 public:
  Board(WallSensorHAL *wall_sensor, IMUHAL *imu)
      : m_wall_sensor(wall_sensor),
        m_imu(imu) {
    std::cout << "Board created - " << STR(BOARD_NAME) << std::endl;
  };

  void initializeFeatures() {
    std::cout << "Initializing features" << std::endl;
    if (m_wall_sensor) {
      m_wall_sensor->initialize();
    }
    if (m_imu) {
      m_imu->initialize();
    }
  }

  void performActions() {
    std::cout << "Performing actions" << std::endl;
    float distance = 0;
    float yaw_rate = 0;
    if (m_wall_sensor) {
      distance = m_wall_sensor->getDistance();
    }
    if (m_imu) {
      /// this lets me do stuff particular to one kind of IMU
      /// where it only exists for that class
      if (auto digitalIMU = dynamic_cast<DigitalIMU *>(m_imu)) {
        digitalIMU->calibrate();
      }
      yaw_rate = m_imu->getYawRate();
    }
    std::cout << "Distance: " << distance << std::endl;
    std::cout << "Yaw rate: " << yaw_rate << std::endl;
  }

 private:
  WallSensorHAL *m_wall_sensor = nullptr;
  IMUHAL *m_imu = nullptr;
};

////////////////////////////////////////////////

WallSensorHAL *setupSensors() {
  WallSensorHAL *p_sensor = nullptr;
#ifdef WALLSENSOR_PRESENT
#ifdef IR_SENSOR
  static ReflectiveIRSensor reflectiveIRSensor;
  p_sensor = &reflectiveIRSensor;
#endif
#ifdef TOF_SENSOR
  static TOFSensor tofSensor;
  p_sensor = &tofSensor;
#endif
#endif
  return p_sensor;
}

IMUHAL *setupIMU() {
  IMUHAL *p_imu = nullptr;
#ifdef IMU_PRESENT
#ifdef DIGITAL_IMU
  static DigitalIMU digitalIMU;
  p_imu = &digitalIMU;
#endif
#ifdef ANALOGUE_IMU
  static AnalogIMU analogueIMU;
  p_imu = &analogueIMU;
#endif
#endif
  return p_imu;
}

int main() {
  WallSensorHAL *wall_sensor = setupSensors();
  IMUHAL *imu = setupIMU();

  Board board(wall_sensor, imu);

  board.initializeFeatures();
  board.performActions();

  return 0;
}
