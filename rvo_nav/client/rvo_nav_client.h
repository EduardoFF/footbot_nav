#ifndef _RVONAVCLIENT_H_
#define _RVONAVCLIENT_H_

#include <iostream>
#include <fstream>

#include <argos2/common/control_interface/ci_controller.h>
#include <argos2/common/utility/logging/argos_log.h>
#include <argos2/common/utility/argos_random.h>
#include <argos2/common/utility/datatypes/color.h>

#include <argos2/common/control_interface/swarmanoid/footbot/ci_footbot_wheels_actuator.h>
#include <argos2/common/control_interface/swarmanoid/footbot/ci_footbot_leds_actuator.h>
#include <argos2/common/control_interface/swarmanoid/ci_range_and_bearing_actuator.h>
#include <argos2/common/control_interface/swarmanoid/footbot/ci_footbot_beacon_actuator.h>

#include <argos2/common/control_interface/swarmanoid/ci_range_and_bearing_sensor.h>

/* Definition of the foot-bot proximity sensor */
#include <argos2/common/control_interface/swarmanoid/footbot/ci_footbot_proximity_sensor.h>

#include <argos2/common/utility/argos_random.h>


#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <termios.h>
#include <math.h>

/** LCM engine */
#include "lcm/lcmthread.h"

/** NAVIGATION AND AVOIDING COLLISION */
/* Navigations agents */
#include "navigation/Agent.h"
#include "navigation/ORCAAgent.h"
#include "navigation/HRVOAgent.h"
#include "navigation/HLAgent.h"
///Additional sensors
#include <argos2/common/control_interface/swarmanoid/footbot/ci_footbot_encoder_sensor.h>
#include <argos2/common/control_interface/ci_wifi_sensor.h>
#include <argos2/common/control_interface/ci_wifi_actuator.h>

/* Obstacle radius */
#define OBSTACLE_RADIUS 0.12
#define ODOMETRY_CORRECTION_FACTOR_RIGHT 1
#define ODOMETRY_CORRECTION_FACTOR_LEFT 1


using namespace argos;
using namespace std;

#include <argos2/common/utility/datatypes/datatypes.h>

/// use waypoint lists instead of single waypoint for control
#define USE_WP_LIST
#ifdef USE_WP_LIST
#include "lcm/timestamped_waypoint_list_handler.h"
#endif

#define NODE_VALIDITY_THRESHOLD 1000

class RVONavClient
{
 private:


  //< State of the robot
  typedef 
    enum RobotNavState 
  {
    STATE_INIT,
    STATE_INIT_READY,
    STATE_MOVING, 
    STATE_ARRIVED_AT_TARGET, 
    STATE_ORIENTING
  } RobotNavState;


  typedef 
    enum LedsControlMode
  {
    LEDS_NONE,
    LEDS_MOBILITY,
    LEDS_FIXED,
  } LedsControlMode;

  typedef 
    enum BeaconControlMode
  {
    BEACON_NONE,
    BEACON_MOBILITY,
    BEACON_MOBILITY_LETTER_DEMO,
    BEACON_FIXED,
  } BeaconControlMode;
  
  UInt8 m_myID;
  CCI_Robot* m_robot;
  CARGoSRandom::CRNG* RandomGen;
  UInt32 RandomSeed;
  std::string m_MyIdStr;
  UInt64 m_Steps;

  RobotNavState m_state;
  LedsControlMode m_ledMode;
  BeaconControlMode m_beaconMode;
  CColor m_ledFixedColor;
  CColor m_beaconFixedColor;

  //! Sensors:
  CCI_RangeAndBearingSensor* m_rangeAndBearingSensor;

  //! Actuators:
  CCI_FootBotWheelsActuator* m_pcWheels;
  CCI_FootBotLedsActuator *m_ledsActuator;
  CCI_RangeAndBearingActuator* rangeAndBearingActuator;
  CCI_FootBotBeaconActuator* m_beaconActuator;

  // Internal functionality:
  //  RobotAddressType RobotIdToAddress(const string& id);
  //  RobotAddressType RobotIdToAddress(const char *id);


  void setLeds();
  void setBeacon();


  /** LCM engine */
  // LCM thread
  LCMThread lcmThread;

  /// make it static because in simulation robots can share the same config handler
  //TODO: enable online configuration
  //  static TimestampedConfigMsgHandler *m_configHandler;
  void processConfig(std::string);
#ifndef USE_WP_LIST
  // LCM thread command channel
  LCMThread lcmThreadCommand;
#else
  static TimestampedWaypointListHandler *m_wpControl;
#endif
  /// List of node obstacles retrieved by LCM-tracking system. The
  /// other nodes are an obstacles for me.
  map<UInt8, Node> listNodeObstacles;
  // Myself Node
  Node mySelf;

  /// NAVIGATION AND AVOIDING COLLISION 
  /// Additional sensors 
  CCI_FootBotEncoderSensor* encoderSensor;

  /// Navigation agents 
  Agent * agent;
  HLAgent hlAgent;
  ORCAAgent orcaAgent;
  HRVOAgent hrvoAgent;

  /// Local navigation
  std::string localNavigationType;
  int localNavigationIndex;

  /// Mobility parameters 
  Real axisLength;
  CVector2 position;
  CVector2 velocity;
  CRadians angle;
  CRadians angularSpeed;
  CVector2 deltaPosition;
  Real speed;
  Real m_targetMinPointDistance;
  Real m_targetOrientation; //! in degrees
  bool m_doTargetOrientation;

  /* Target -> next (x,y) point */
  CVector2 m_targetPosition;

  // Area bounds as fixed obstacles
  CVector2 originAreaCoord;
  CVector2 destinationAreaCoord;

  Real originAreaX;
  Real originAreaY;
  Real destinationAreaX;
  Real destinationAreaY;

  /** PROXIMITY SENSORS */
  /* Pointer to the foot-bot proximity sensor */
  CCI_FootBotProximitySensor* m_pcProximity;
  /* Maximum tolerance for the angle between
   * the robot heading direction and
   * the closest obstacle detected. */
  CDegrees m_cAlpha;
  /* Maximum tolerance for the proximity reading between
   * the robot and the closest obstacle.
   * The proximity reading is 0 when nothing is detected
   * and grows exponentially to 1 when the obstacle is
   * touching the robot.
   */
  Real m_fDelta;
  /* Wheel speed. */
  Real m_fWheelVelocity;
  /* Angle tolerance range to go straight.
   * It is set to [-alpha,alpha]. */
  CRange<CRadians> m_cGoStraightAngleRange;
 public:

  /* Class constructor. */
  RVONavClient(UInt8 robot_id, CCI_Robot &);

  /* Class destructor. */
  virtual ~RVONavClient() {
  }

  /// state 
  void setState(RobotNavState s);

  /** NAVIGATION AND AVOIDING COLLISION */
  /* Navigation agents */
  void setAgent(Agent &a);
  void updateAgent(Agent *a);

  /* Local navigation */
  void initLocalNavigation(TConfigurationNode& t_tree);
  void initGlobalNavigation(TConfigurationNode& t_tree);
  void initOdometry();
#ifndef FOOTBOT_LQL_SIM
    static UInt64 getTime();
#else
    UInt64 getTime();
#endif

  static std::string getTimeStr();

  static bool isNodeValid( Node &n );
  /* Obstacles */
  void addNodesAsObstacles(map<UInt8, Node> listNodeObstacles);
  std::vector<CVector2> addAreaBounds(CVector2 bottomLeftCoord, CVector2 upperRightCoord);
  void setFixedObstacles(std::vector<CVector2> obstaclesPoints);

  /* Update navigation */
  void updateNavigation();
  void updateDesideredVelocity();
  void updateVelocity();

  /** PROXIMITY SENSORS */
  void updateVelocityProximitySensors();

  void init(TConfigurationNode& t_tree);

  void update();
  void destroy();

  static UInt8 ConvertValueToByte(Real value);
  static Real ConvertByteToValue(UInt8 byte);
};

#endif
