#include <iostream>

/// this would go in a "board-config.h" file

/***
 * A config file can hold a bunch of preprocessor macros that define
 * the hardware configuration of the target board
 */

// pick your board
// #define BOARD_A
#define BOARD_B

// Board A configuration
#ifdef BOARD_A
constexpr char BOARD_NAME[] = "D4";
#define HAS_WALLSENSOR 1
#define HAS_IR_SENSOR 0
#define HAS_TOF_SENSOR 1

#define HAS_IMU 1
#define HAS_ANALOGUE_IMU 1
#define HAS_DIGITAL_IMU 0
#endif
//
//// Board B configuration
#ifdef BOARD_B
constexpr char BOARD_NAME[] = "MR32";

#define HAS_WALLSENSOR 1
#define HAS_IR_SENSOR 1
#define HAS_TOF_SENSOR 0

#define HAS_IMU 1
#define HAS_ANALOGUE_IMU 0
#define HAS_DIGITAL_IMU 1
#endif

/////////////////////////////////////////////////

/***
 * start off with some hardware abstraction models
 * Any concrete hardware implementation can be used so long
 * as it implements the interface. Yopu cannot instantiate the
 * interface classes.
 * Derived classes can add functionality.
 * Only a bare minimum interface is shown here to keep it simple
 */
class WallSensorHAL {
 public:
  virtual void initialize() = 0;
  virtual float getDistance() = 0;
  virtual ~WallSensorHAL() = default;
};

/// Here are a couple of actual sensor types
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

/***
 * Same story with the IMU devices. The concrete implementations
 * give the same kind of results with the same interface but
 * would be expected to do it differently
 *
 */
class IMUHAL {
 public:
  virtual void initialize() = 0;
  virtual float getYawRate() = 0;
  virtual ~IMUHAL() = default;
};

/// lets give the digital IMU an extra method
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

/***
 * The board is a collection of hardware abstractions. Here
 * we assume that the board has both a Wallsensor and an IMU.
 *
 * We give the board pointers to the actual instances to be used.
 *
 * By making those pointers to the base type, we can pass in any
 * concrete implementation.
 *
 * The pointers can be null if there is not a sensor or IMU
 */
class Board {
 public:
  /// The constructor assigns the pointer for each feature
  /// There are other ways to do this for long feature lists
  /// It is best to have the board initialize all features
  Board(WallSensorHAL *wall_sensor, IMUHAL *imu)
      : m_wall_sensor(wall_sensor),
        m_imu(imu) {
    std::cout << "Board created - " << BOARD_NAME << std::endl;
    initializeFeatures();
    std::cout << "Board ready\n" << std::endl;
  };

  /***
   * This could be the hardware setup or it could be state configuration
   * See below for where the objects are actually created. Hardware config
   * should probably done there.
   */
  void initializeFeatures() {
    std::cout << "Initializing features" << std::endl;
    if (m_wall_sensor) {
      m_wall_sensor->initialize();
    }
    if (m_imu) {
      m_imu->initialize();
    }
  }

  /***
   * This might be a good place to check that everything ois good to go
   * but instead we will just use the feaatures we have.
   * The guards ensure that we do not try to call null objects.
   * Remember to have safe defaults in those cases.
   */
  void performActions() {
    std::cout << "Performing actions" << std::endl;
    float distance = 0;
    float yaw_rate = 0;
    /// Notice that we do not care now what kind of concrete
    /// implementation we have. Just call against the interface.
    if (m_wall_sensor) {
      distance = m_wall_sensor->getDistance();
    }
    if (m_imu) {
      /// this lets me do stuff particular to one kind of IMU
      /// where it only exists for that class. Not that
      /// calibrate is a good candidate here.
      /// Note that RTTI may not be available on some platforms
      ///      so you may need to add the extra method to the base class.
      if (auto digitalIMU = dynamic_cast<DigitalIMU *>(m_imu)) {
        digitalIMU->calibrate();
      }
      yaw_rate = m_imu->getYawRate();
    }
    std::cout << std::endl;
    std::cout << "Distance: " << distance << std::endl;
    std::cout << "Yaw rate: " << yaw_rate << std::endl;
  }

 private:
  WallSensorHAL *m_wall_sensor = nullptr;
  IMUHAL *m_imu = nullptr;
};

////////////////////////////////////////////////

/***
 * In this section we get to actually create the devices described by
 * the configuration file.
 *
 * The conditional comppilation is scruffy but it works. There are other
 * ways to do this if it offends.
 *
 * The actual instances are created as static objectsin the setup
 * functions.
 *
 * Static variables are held in a special area of RAM and the space needed
 * is allocated at compile time. That means that, by the time your code
 * runs, the memory is already allocated.
 *
 * Being static, they are not destroyed when the fucntion exits but we
 * can still access them so long as we remember their address. To  do
 * that, the setup methods return a pointer to the object. The pointer
 * type is the base class type so that we can create objects of any
 * concrete type.
 *
 * It is probably a good itea to do at least some of the initialization
 * here. If using the STM32CubeMX, most of that will be done automatically
 * and we just get to connect stuff up to the pointers.
 */

WallSensorHAL *setupWallSensors() {
  WallSensorHAL *p_sensor = nullptr;
#if HAS_WALLSENSOR
#if HAS_IR_SENSOR
  static ReflectiveIRSensor reflectiveIRSensor;
  p_sensor = &reflectiveIRSensor;
#elif HAS_TOF_SENSOR
  static TOFSensor tofSensor;
  p_sensor = &tofSensor;
#else
#warning "No wall sensor selected"
#endif
#endif
  return p_sensor;
}

IMUHAL *setupIMU() {
  IMUHAL *p_imu = nullptr;
#if HAS_IMU
#if HAS_ANALOGUE_IMU
  static DigitalIMU digitalIMU;
  p_imu = &digitalIMU;
#elif HAS_DIGITAL_IMU
  static AnalogIMU analogueIMU;
  p_imu = &analogueIMU;
#else
#warning "No IMU selected"
#endif
#endif
  return p_imu;
}

/***
 * Finally we are good to go. We first create all the
 * hardware features, and remember their addresses.
 * Then we give those addresses to the board and it does
 * its thing.
 * None of the objects are created in dynamic memory so
 * there is no need to worry about memory management.
 *
 */
int main() {
  WallSensorHAL *wall_sensor = setupWallSensors();
  IMUHAL *imu = setupIMU();

  Board board(wall_sensor, imu);

  board.performActions();

  return 0;
}
