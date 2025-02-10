#include <iostream>

/// this would go in a "board-config.h" file

/***
 * A config file can hold a bunch of configurations that define
 * the hardware configuration of the target board
 */

// Configuration structure
struct BoardConfig {
  const char *boardName = "UNDEFINED";

  bool hasWallSensor = false;
  bool hasIrSensor = false;
  bool hasTofSensor = false;

  bool hasImu = false;
  bool hasAnalogImu = false;
  bool hasDigitalImu = false;
};

// Define configurations for different boards
constexpr BoardConfig BoardA_Config = {
    .boardName = "D4",      //
    .hasWallSensor = true,  //
    .hasTofSensor = true,   //
    .hasImu = true,         //
    .hasDigitalImu = false  //
};

constexpr BoardConfig BoardB_Config = {
    .boardName = "MR32",    //
    .hasWallSensor = true,  //
    .hasIrSensor = true,    //
    .hasImu = true,         //
    .hasDigitalImu = true   //
};

// Select the board
constexpr BoardConfig CurrentBoard = BoardB_Config;

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
    std::cout << "Board created - " << CurrentBoard.boardName << std::endl;
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
      ///      so you may need to add the extra method(s) to the base class.
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
 * The compiler, or linter, because it knows the state of the flags, may
 * complain about unreachable code here
 *
 * It is probably a good idea to do at least some of the initialization
 * here. If using the STM32CubeMX, most of that will be done automatically
 * and we just get to connect stuff up to the pointers.
 */

WallSensorHAL *setupWallSensors() {
  if constexpr (CurrentBoard.hasWallSensor) {
    if constexpr (CurrentBoard.hasIrSensor) {
      static ReflectiveIRSensor irSensor;
      return &irSensor;
    } else if constexpr (CurrentBoard.hasTofSensor) {
      static TOFSensor tofSensor;
      return &tofSensor;
    }
  }
  return nullptr;
}

IMUHAL *setupIMU() {
  if constexpr (CurrentBoard.hasImu) {
    if constexpr (CurrentBoard.hasAnalogImu) {
      static AnalogIMU analogIMU;
      return &analogIMU;
    } else if constexpr (CurrentBoard.hasDigitalImu) {
      static DigitalIMU digitalIMU;
      return &digitalIMU;
    }
  }
  return nullptr;
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
