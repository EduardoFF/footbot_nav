#include "rvo_nav_client.h"


/// to handle debug and report macros
#define STR_EXPAND(tok) #tok
#define STR(tok) STR_EXPAND(tok)

#define __USE_DEBUG_NAV 0
#define __USE_DEBUG_ERROR 1

#if __USE_DEBUG_NAV
#define DEBUGNAV(m, ...) \
{\
  std::string t_str = getTimeStr();\
  fprintf(stderr, "%s DEBUGNAV[%d]: " m,\
	  t_str.c_str(),\
	  (int) m_myID,\
          ## __VA_ARGS__);\
  fflush(stderr);\
}
#else
#define DEBUGNAV(m, ...)
#endif

#if __USE_DEBUG_ERROR
#define DEBUGERROR(m, ...) \
{\
  fprintf(stderr, "DEBUGERROR[%d]: " m,\
	  (int) m_myID,\
          ## __VA_ARGS__);\
  fflush(stderr);\
}
#else
#define DEBUGERROR(m, ...)
#endif


//TimestampedConfigMsgHandler *MARSFootbot::m_configHandler =
//  new TimestampedConfigMsgHandler("udpm://239.255.76.67:7667?ttl=1", "CONFIG", true);

#ifdef USE_WP_LIST
TimestampedWaypointListHandler *RVONavClient::m_wpControl =
  new TimestampedWaypointListHandler("udpm://239.255.76.67:7667?ttl=1", 
				 "TARGET", true);
#endif


RVONavClient::RVONavClient(UInt8 robot_id, CCI_Robot &robot) :
  m_myID(robot_id),
  RandomSeed(12345),
  m_Steps(0),
  m_state(STATE_INIT),
  m_pcWheels(NULL), 
  m_ledsActuator(NULL), 
  m_targetMinPointDistance(0.12f), 
  m_pcProximity(NULL), 
  m_cAlpha(10.0f), 
  m_fDelta(1.0f), 
  m_fWheelVelocity(5.0f), 
  m_cGoStraightAngleRange(-ToRadians(m_cAlpha), 
			  ToRadians(m_cAlpha)) 
{
  m_robot = &robot;
  m_ledMode=LEDS_MOBILITY;
  m_beaconMode = BEACON_NONE;
  m_doTargetOrientation = false;
  m_targetOrientation = 0.0;
}


  bool 
RVONavClient::isNodeValid( Node &n )
{
#ifndef FOOTBOT_LQL_SIM
  UInt64 dt = getTime() - n.getTimestamp();
  return (dt < NODE_VALIDITY_THRESHOLD);
#else
  /// in simulation, assume everything is valid
  /// i.e., we do not lose track of the robots
  return true;
#endif
}

void 
RVONavClient::setLeds()
{
  if( m_ledMode == LEDS_MOBILITY )
  {
    switch (m_state )
    {
    case STATE_INIT_READY:
      m_ledsActuator->SetAllColors(CColor::BLUE);
      break;
    case STATE_ARRIVED_AT_TARGET:
      m_ledsActuator->SetAllColors(CColor::RED);
      break;
    case STATE_MOVING:
      m_ledsActuator->SetAllColors(CColor::GREEN);
      break;
    case STATE_ORIENTING:
      m_ledsActuator->SetAllColors(CColor::YELLOW);
      break;
    default:
      break;
    }
  }
  else if( m_ledMode == LEDS_FIXED )
  {
    m_ledsActuator->SetAllColors(m_ledFixedColor);
  }
}

void
RVONavClient::setBeacon()
{
  if( m_beaconMode == BEACON_MOBILITY || 
      m_beaconMode == BEACON_MOBILITY_LETTER_DEMO )
  {
    switch (m_state )
    {
    case STATE_INIT_READY:
      m_beaconActuator->SetColor(CColor::BLUE);
      break;
    case STATE_ARRIVED_AT_TARGET:
      if( m_beaconMode == BEACON_MOBILITY_LETTER_DEMO )
      {
	/// check if it is outside area
	position = CVector2(mySelf.getPosition().GetX(), mySelf.getPosition().GetY());
	if( position.GetX() < -0.3 ||
	    position.GetY() < -0.3 )
	{
	  m_beaconActuator->SetColor(CColor::BLACK);
	}
	else
	{
	  m_beaconActuator->SetColor(CColor::RED);
	}
      }
      else
      {
	m_beaconActuator->SetColor(CColor::RED);
      }
      break;
    case STATE_MOVING:
      m_beaconActuator->SetColor(CColor::GREEN);
      break;
    case STATE_ORIENTING:
      m_beaconActuator->SetColor(CColor::YELLOW);
      break;
    default:
      break;
    }
  } 

}


  void 
RVONavClient::setState(RobotNavState s)
{
  if( s == m_state )
    return;
  /// change of state
  m_state = s;

}

  void 
RVONavClient::init(TConfigurationNode& t_node) 
{
  /// Get actuators and sensors
  m_pcWheels = dynamic_cast<CCI_FootBotWheelsActuator*>(m_robot->GetActuator("footbot_wheels"));
  m_ledsActuator = dynamic_cast<CCI_FootBotLedsActuator*>(m_robot->GetActuator("footbot_leds"));
  m_pcProximity = dynamic_cast<CCI_FootBotProximitySensor*>(m_robot->GetSensor("footbot_proximity"));
  //rangeAndBearingActuator = 
  //dynamic_cast<CCI_RangeAndBearingActuator*> (m_robot->GetActuator("range_and_bearing"));
  m_beaconActuator          = dynamic_cast<CCI_FootBotBeaconActuator*>   (m_robot->GetActuator("footbot_beacon"));
  //rangeAndBearingSensor   = 
  //dynamic_cast<CCI_RangeAndBearingSensor*>   (GetRobot().GetSensor  ("range_and_bearing"));

  /// Random
  GetNodeAttributeOrDefault(t_node, "RandomSeed", RandomSeed, RandomSeed);

  /// distance threshold to determine that robot reached target point
  //GetNodeAttributeOrDefault(t_node, "targetMinPointDistance", m_targetMinPointDistance, m_targetMinPointDistance);
  std::string text;

  if (NodeExists(t_node, "targetMinPointDistance")) 
  {
    GetNodeText(GetNode(t_node, "targetMinPointDistance"), text);
    sscanf(text.c_str(), "%f", &m_targetMinPointDistance);
    //m_targetMinPointDistance = fmin(MAX_RESOLUTION, m_targetMinPointDistance);
  }

  DEBUGNAV("Min distance to target point %f in robot %s\n", 
	   m_targetMinPointDistance, GetRobot().GetRobotId().c_str());

  if (NodeExists(t_node, "originAreaX")) 
  {
    GetNodeText(GetNode(t_node, "originAreaX"), text);
    sscanf(text.c_str(), "%f", &originAreaX);
  }

  if (NodeExists(t_node, "originAreaY")) 
  {
    GetNodeText(GetNode(t_node, "originAreaY"), text);
    sscanf(text.c_str(), "%f", &originAreaY);
  }

  if (NodeExists(t_node, "destinationAreaX")) {
    GetNodeText(GetNode(t_node, "destinationAreaX"), text);
    sscanf(text.c_str(), "%f", &destinationAreaX);
  }

  if (NodeExists(t_node, "destinationAreaY")) {
    GetNodeText(GetNode(t_node, "destinationAreaY"), text);
    sscanf(text.c_str(), "%f", &destinationAreaY);
  }

  DEBUGNAV("Area bounds-> ORIGIN (%f,%f), DESTINATION (%f,%f)\n", 
	   originAreaX, originAreaY, destinationAreaX, destinationAreaY);
  originAreaCoord.SetX(originAreaX);
  originAreaCoord.SetY(originAreaY);
  destinationAreaCoord.SetX(destinationAreaX);
  destinationAreaCoord.SetY(destinationAreaY);

  CARGoSRandom::CreateCategory("local_rng", RandomSeed);
  RandomGen = CARGoSRandom::CreateRNG("local_rng");

  printf("checking for led conf\n");
  if (NodeExists(t_node, "ledcontrol")) 
  {
    printf("NODE EXISTS LEDS\n");
    TConfigurationNode node = GetNode(t_node, "ledcontrol");
    string ledMode("DEFAULT");
    GetNodeAttributeOrDefault<std::string>(node, "mode", ledMode, ledMode);
    if ( ledMode == "DEFAULT" )
      m_ledMode = LEDS_NONE;
    else if ( ledMode == "MOBILITY" )
      m_ledMode = LEDS_MOBILITY;
    else if ( ledMode == "FIXED" )
      m_ledMode = LEDS_FIXED;
    else
      m_ledMode = LEDS_NONE;

    string ledFixedColor("black");
    GetNodeAttributeOrDefault<std::string>(node, "fixedColor", 
					   ledFixedColor, ledFixedColor);
    stringstream ss(ledFixedColor);
    ss >> m_ledFixedColor;
    cout << "ledMode " << ledMode << " fcolor " << ledFixedColor << endl;
  }
  else
  {
    printf("NO LED CONF WTF\n");
  }

  printf("checking for beacon conf\n");
  if (NodeExists(t_node, "beaconcontrol")) 
  {
    printf("NODE EXISTS BEACON\n");
    TConfigurationNode node = GetNode(t_node, "beaconcontrol");
    string beaconMode("DEFAULT");
    GetNodeAttributeOrDefault<std::string>(node, "mode", beaconMode, beaconMode);
    if ( beaconMode == "DEFAULT" )
      m_beaconMode = BEACON_NONE;
    else if ( beaconMode == "MOBILITY" )
      m_beaconMode = BEACON_MOBILITY;
    else if ( beaconMode == "MOBILITY_LETTER_DEMO" )
      m_beaconMode = BEACON_MOBILITY_LETTER_DEMO;
    else if ( beaconMode == "FIXED" )
      m_beaconMode = BEACON_FIXED;
    else
      m_beaconMode = BEACON_NONE;

    string beaconFixedColor("black");
    GetNodeAttributeOrDefault<std::string>(node, "fixedColor", 
					   beaconFixedColor, beaconFixedColor);
    stringstream ss(beaconFixedColor);
    ss >> m_beaconFixedColor;
    cout << "beaconMode " << beaconMode << " fcolor " << beaconFixedColor << endl;
  }
  else
  {
    printf("NO beacon CONF WTF\n");
  }

  fflush(stdout);
  setLeds();
  setBeacon();


  /** LCM engine */
  /// LCM Thread
  std::string track_chan("TRACK");
  std::string target_chan("TARGET");
  if (NodeExists(t_node, "lcm")) 
  {
    TConfigurationNode node = GetNode(t_node, "lcm");
    GetNodeAttributeOrDefault<std::string>(node, "track", track_chan, track_chan);
  }
  else
  {
    printf("NO LCM CONF\n");
  }
  printf("Listening to %s\n", track_chan.c_str());
#ifndef USE_WP_LIST
  lcmThreadCommand.setLCMEngine("udpm://239.255.76.67:7667?ttl=1", "TARGET");
  lcmThreadCommand.startInternalThread();
#else
  //m_wpControl = 
  //new TimestampedWaypointListHandler("udpm://239.255.76.67:7667?ttl=1", 
  //	       "TARGET");
//  if( !m_wpControl->run() )
//    DEBUGERROR("error while running waypoint control\n");
#endif

  lcmThread.setLCMEngine("udpm://239.255.76.67:7667?ttl=1", track_chan.c_str());
  lcmThread.startInternalThread();
  /** NAVIGATION AND AVOIDING COLLISION */
  /* Additional sensors */
  // we do not have encoderSensor in simulation
  encoderSensor = dynamic_cast<CCI_FootBotEncoderSensor*>(m_robot->GetSensor("footbot_encoder"));

 
  /// Init navigation methods 
  initOdometry();
  initLocalNavigation(t_node);

  setState(STATE_INIT_READY);
}

void
RVONavClient::processConfig(std::string msg)
{
  std::stringstream ss(msg);
  std::string cmd;
  ss >> cmd;
  cout << "got config cmd " << cmd << endl;

}

  void 
RVONavClient::update() 
{

  CRadians oA;
  CVector3 axis;
  
  m_Steps+=1;
#if LCM_CONFIG_ENABLED
  std::pair< bool, TimestampedConfigMsg> c_msg =
    m_configHandler->popNextConfigMsg(m_myID, getTime());
  //printf("RobotId %d ControlStep %lld - config msgs? %d\n",
  //	 m_myID, m_Steps, c_msg.first);
  if( c_msg.first )
  {
    //cout << "robot " << m_myID <<  " time "
    //	 << getTime() << "config? " << c_msg.first
    //	 << " msg.timestamp: " << c_msg.second.timestamp << endl;
    processConfig(c_msg.second.msg);
  }
#endif
  
  /// New target point from the COMMAND engine
#ifdef USE_WP_LIST
  std::pair< bool, TimestampedWaypoint> c_wp =
    m_wpControl->getNextWaypoint(m_myID,getTime());
  if( c_wp.first )
  {
    CVector2 n_wp(c_wp.second.second.GetX(), c_wp.second.second.GetY());
    Real n_ori = c_wp.second.second.GetYaw();
    if( m_targetPosition != n_wp || m_targetOrientation != n_ori)    
    {
      DEBUGNAV("NEW_WP (%f,%f):%d valid from %llu\n", 
	       n_wp.GetX(), n_wp.GetY(), n_ori, c_wp.second.first);
      m_targetPosition.Set( n_wp.GetX(), n_wp.GetY());
      m_targetOrientation = n_ori;
    }
  } 
  else
  {
    /// just stay in same pos
    if (lcmThread.getLcmHandler()->existNode(m_myID)) 
    {
      Node nodeCommand = 
	lcmThread.getLcmHandler()->getNodeById(m_myID);
      m_targetPosition.Set(nodeCommand.getPosition().GetX(), 
			 nodeCommand.getPosition().GetY());

      nodeCommand.getOrientation().ToAngleAxis(oA, axis);
      if (axis.GetZ() < 0)
	oA = -oA;
      m_targetOrientation = ToDegrees(oA).GetValue();
      DEBUGNAV("REMAIN_IN_POS: (%f,%f): %f\n", 
	     m_targetPosition.GetX(), 
	     m_targetPosition.GetY(),
	     m_targetOrientation);
    }
  }
#else
  if (lcmThreadCommand.getLcmHandler()->existNode(m_myID)) 
  {
    Node nodeCommand = 
      lcmThreadCommand.getLcmHandler()->getNodeById(m_myID);
    m_targetPosition.Set(nodeCommand.getPosition().GetX(), 
		       nodeCommand.getPosition().GetY());
    DEBUGNAV("ID %d - SET_TARGET: (%f,%f)\n", 
	   (int) m_myID, 
	   nodeCommand.getPosition().GetX(), 
	   nodeCommand.getPosition().GetY());
  } 
  else 
  {
    if (lcmThread.getLcmHandler()->existNode(m_myID)) 
    {
      Node nodeCommand = 
	lcmThread.getLcmHandler()->getNodeById(m_myID);
      m_targetPosition.Set(nodeCommand.getPosition().GetX(), 
			 nodeCommand.getPosition().GetY());
      DEBUGNAV("ID %d - REMAIN_IN_POS: (%f,%f)\n", 
	     (int) m_myID, 
	     nodeCommand.getPosition().GetX(), 
	     nodeCommand.getPosition().GetY());
    }
  }
#endif

  //DEBUGNAV("Min distance: %f\n", m_targetMinPointDistance);

  /*  */
  if (lcmThread.getLcmHandler()->existNode(m_myID)) 
  {
    DEBUGNAV("lcmThread has my INFO (%d)\n", m_myID);

#if __USE_DEBUG_NAV
    lcmThread.getLcmHandler()->printNodeListElements();
#endif

    /** LCM related Node information */
    //To get the other nodes locations through LCM
    listNodeObstacles = lcmThread.getLcmHandler()->retrieveNodeList();

    //DEBUGNAV("got NodeList\n");
    //Get myself

    //mySelf = lcmThread.getLcmHandler()->getNodeById(0);
    mySelf = lcmThread.getLcmHandler()->getNodeById(m_myID);
    //DEBUGNAV("Got myself\n");

    //DEBUGNAV("ID %d\n", (int) mySelf.getId());
    //DEBUGNAV("POS (%f,%f)\n", mySelf.getPosition().GetX(), mySelf.getPosition().GetY());
    //DEBUGNAV("QUAT (%f,%f,%f,%f)\n", mySelf.getOrientation().GetW(), mySelf.getOrientation().GetX(), mySelf.getOrientation().GetY(), mySelf.getOrientation().GetZ());
    //DEBUGNAV("VEL %d\n", mySelf.getVelocity());

    mySelf.getOrientation().ToAngleAxis(oA, axis);
    //DEBUGNAV("0...\n");
    if (axis.GetZ() < 0)
      oA = -oA;
    //DEBUGNAV("1...\n");

    //DEBUGNAV("[CONTROLLER] ID %d - ORIENTATION: (yaw) - (%f) - (%fº)\n", 
	   //m_myID, oA.GetValue(), ToDegrees(oA).GetValue());

    //DEBUGNAV("[CONTROLLER] AN %d - Previous (%f,%f) - New (%f,%f)\n", 
	   //m_myID, mySelf.getPosition().GetX(), 
	   //mySelf.getPosition().GetY(), 
	   //m_targetPosition.GetX(), 
	   //m_targetPosition.GetY());

    // Position
    position = CVector2(mySelf.getPosition().GetX(), mySelf.getPosition().GetY());
    // Angle
    angle = oA;

    // Velocity
    //velocity = CVector2(speed, 0);
    velocity = CVector2(mySelf.getVelocity(), 0);

    /** NAVIGATION AND AVOIDING COLLISION */

    updateAgent(agent);
    agent->clearObstacles();
    addNodesAsObstacles(listNodeObstacles);

    if ((m_targetPosition - position).Length() < m_targetMinPointDistance) 
    {
      if( m_doTargetOrientation )
      {
	Real oAd = ToDegrees(oA).GetValue();
	if ( oAd < 0 )
	  oAd = 360.0 + oAd;
	/// now, make them between -PI and PI
	if( oAd > 180.0)
	  oAd = oAd - 360.0;
	/// distance between angles
	Real cdev = m_targetOrientation - oAd;
	cdev += (cdev>180) ? -360 : (cdev<-180) ? 360 : 0;
	//Real cdev = fabs(m_targetOrientation - oAd);
	//printf("STATE_ORIENTING cdev %f\n", cdev);
	//printf("current orient %f target orient %f cdev %f\n", 
	 //      oAd, m_targetOrientation, cdev );
	if( fabs(cdev)  < 5.0 ) 
	  setState( STATE_ARRIVED_AT_TARGET );
	else
	  setState( STATE_ORIENTING );
      }	
      else
      {
	setState( STATE_ARRIVED_AT_TARGET );
      }
      DEBUGNAV("State: STOPPED\n");
    } 
    else 
    {
      setState( STATE_MOVING );
      DEBUGNAV("State: MOVING\n");
    }
    updateNavigation();
  }
}



  void 
RVONavClient::initLocalNavigation(TConfigurationNode& t_node) 
{
  hlAgent.Init(t_node);
  hlAgent.axisLength = axisLength;
  orcaAgent.Init(t_node);
  orcaAgent.axisLength = axisLength;
  hrvoAgent.Init(t_node);
  hrvoAgent.axisLength = axisLength;

  localNavigationType = "HL";

  if (NodeExists(t_node, "local_navigation")) 
  {
    TConfigurationNode node = GetNode(t_node, "local_navigation");
    if (node.HasAttribute("type"))
      GetNodeAttribute(node, "type", localNavigationType);
    GetNodeAttributeOrDefault(node, "doTargetOrientation", 
			      m_doTargetOrientation, 
			      m_doTargetOrientation);
    GetNodeAttributeOrDefault(node, "targetOrientation", 
			      m_targetOrientation, 
			      m_targetOrientation);

  }

  if (localNavigationType == "HL") 
  {
    localNavigationIndex = 2;
    setAgent(hlAgent);
  } 
  else if (localNavigationType == "ORCA") 
  {
    localNavigationIndex = 0;
    setAgent(orcaAgent);
  } 
  else if (localNavigationType == "HRVO") 
  {
    localNavigationIndex = 0;
    setAgent(hrvoAgent);
  } 
  else 
  {
    throw "Navigation type not defined!";
    return;
  }
}

  void 
RVONavClient::initOdometry() 
{
  position = CVector2(0, 0);
  angle = CRadians(0);
  velocity = CVector2(0, 0);
  axisLength = encoderSensor->GetReading().WheelAxisLength * 0.01;
  DEBUGNAV("INIT axis length %.3f", axisLength);

}

  void 
RVONavClient::setAgent(Agent &a) 
{
  if (agent == &a)
    return;
  agent = &a;
}

  void 
RVONavClient::updateAgent(Agent *a) 
{
  a->position = position;
  a->velocity = velocity;
  a->angularSpeed = angularSpeed;
  //a->leftWheelSpeed=leftWheelSpeed;
  //a->rightWheelSpeed=rightWheelSpeed;
  a->angle = angle;
}

  void 
RVONavClient::addNodesAsObstacles(map<UInt8, Node> listNodeObstacles) 
{
  /// Aux variables
  CVector2 auxPosition;
  CVector2 auxVelocity;

  /// Create a list of Nodes
  for (map<UInt8, Node>::iterator it = listNodeObstacles.begin(); 
       it != listNodeObstacles.end(); it++) 
  {
    // I'm not and obstacle for myself!
    if ((it->second).getId() != mySelf.getId()) 
    {

      //printf("Agent \n");
      auxPosition.Set((it->second).getPosition().GetX(), 
		      (it->second).getPosition().GetY());
      auxVelocity.Set((it->second).getVelocity(), 0);

      // For a dynamic obstacle we need to use the addObstacleAtPoint(position,velocity,radius)
      //agent->addObstacleAtPoint(auxPosition, OBSTACLE_RADIUS);
      agent->addObstacleAtPoint(auxPosition, auxVelocity, OBSTACLE_RADIUS);

      //DEBUGNAV("Adding obstacle : %d\n", (it->second).getId());
    }
  }
  /// Init fixed obstacles
  setFixedObstacles(addAreaBounds(originAreaCoord, destinationAreaCoord));
}

  std::vector<CVector2> 
RVONavClient::addAreaBounds(CVector2 bottomLeftCoord, CVector2 upperRightCoord) 
{

  //All obstacle points
  std::vector<CVector2> obstaclePoints;

  CVector2 upperLeftCoord;
  upperLeftCoord.Set(bottomLeftCoord.GetX(), upperRightCoord.GetY());

  CVector2 bottomRightCoord;
  bottomRightCoord.Set(upperRightCoord.GetX(), bottomLeftCoord.GetY());

  //	printf("Vertices (%f,%f),(%f,%f),(%f,%f),(%f,%f)\n", bottomLeftCoord.GetX(), bottomLeftCoord.GetY(), upperLeftCoord.GetX(), upperLeftCoord.GetY(), upperRightCoord.GetX(), upperRightCoord.GetY(),
  //			bottomRightCoord.GetX(), bottomRightCoord.GetY());

  //Distance from bottom left to upper left.
  CVector2 d1;
  d1 = upperLeftCoord - bottomLeftCoord;

  //	printf("D1 %f\n", d1.Length());

  double numObsD1;
  modf(d1.Length() / (2 * OBSTACLE_RADIUS), &numObsD1);
  double interDistanceD1 = d1.Length() / numObsD1;

  CVector2 pointAux;
  pointAux.SetX(bottomLeftCoord.GetX());
  for (int i = 0; i < numObsD1; i++) {
    pointAux.SetY(bottomLeftCoord.GetY() + i * interDistanceD1);

    //		printf("D1, P(%d) = (%f,%f)\n", i, pointAux.GetX(), pointAux.GetY());
    obstaclePoints.push_back(pointAux);
  }

  //Distance from upper left to upper right
  CVector2 d2;
  d2 = upperLeftCoord - upperRightCoord;
  //printf("D2 %f\n", d2.Length());

  double numObsD2;
  modf(d2.Length() / (2 * OBSTACLE_RADIUS), &numObsD2);
  double interDistanceD2 = d2.Length() / numObsD2;

  pointAux.SetY(upperLeftCoord.GetY());
  for (int i = 0; i < numObsD2; i++) {
    pointAux.SetX(upperLeftCoord.GetX() + i * interDistanceD2);

    //printf("D2, P(%d) = (%f,%f)\n", i, pointAux.GetX(), pointAux.GetY());
    obstaclePoints.push_back(pointAux);
  }

  //Distance from upper right to bottom right
  CVector2 d3;
  d3 = upperRightCoord - bottomRightCoord;
  //printf("D3 %f\n", d3.Length());

  double numObsD3;
  modf(d3.Length() / (2 * OBSTACLE_RADIUS), &numObsD3);
  double interDistanceD3 = d3.Length() / numObsD3;

  pointAux.SetX(upperRightCoord.GetX());
  for (int i = 0; i < numObsD3; i++) {
    pointAux.SetY(upperRightCoord.GetY() - i * interDistanceD3);

    //printf("D3, P(%d) = (%f,%f)\n", i, pointAux.GetX(), pointAux.GetY());
    obstaclePoints.push_back(pointAux);
  }

  //Distance from bottom right to bottom left
  CVector2 d4;
  d4 = bottomRightCoord - bottomLeftCoord;
  //printf("D4 %f\n", d4.Length());

  double numObsD4;
  modf(d4.Length() / (2 * OBSTACLE_RADIUS), &numObsD4);
  double interDistanceD4 = d4.Length() / numObsD4;

  pointAux.SetY(bottomRightCoord.GetY());
  for (int i = 0; i < numObsD4; i++) {
    pointAux.SetX(bottomRightCoord.GetX() - i * interDistanceD4);

    //printf("D4, P(%d) = (%f,%f)\n", i, pointAux.GetX(), pointAux.GetY());
    obstaclePoints.push_back(pointAux);
  }

  return obstaclePoints;

}

  void 
RVONavClient::setFixedObstacles(std::vector<CVector2> obstaclesPoints) 
{
  std::vector<CVector2>::iterator it;
  for (unsigned int i = 0; i < obstaclesPoints.size(); i++) 
  {
    agent->addObstacleAtPoint(obstaclesPoints[i], OBSTACLE_RADIUS);
  }
}

  void 
RVONavClient::updateNavigation() 
{
  if( m_state == STATE_ORIENTING )
  {
    CVector3 axis;
    CRadians oA;
    mySelf.getOrientation().ToAngleAxis(oA, axis);
    if (axis.GetZ() < 0)
      oA = -oA;
    DEBUGNAV("STATE_ORIENTING: Current Orientation %f\n", ToDegrees(oA).GetValue());
    //DEBUGNAV("[CONTROLLER] ID %d - ORIENTATION: (yaw) - (%f) - (%fº)\n", 
	   //m_myID, oA.GetValue(), ToDegrees(oA).GetValue());
    Real oAd = ToDegrees(oA).GetValue();
    if ( oAd < 0 )
      oAd = 360.0 + oAd;
    /// now, make them between -PI and PI
    if( oAd > 180.0)
      oAd = oAd - 360.0;
    /// distance between angles
    Real cdev = m_targetOrientation - oAd;
    cdev += (cdev>180) ? -360 : (cdev<-180) ? 360 : 0;

//    agent->desideredAngle = ToRadians(CDegrees(m_targetOrientation)) - oA;

    agent->desideredAngle = ToRadians(CDegrees(cdev));

    DEBUGNAV("STATE_ORIENTING: desidered_angle %f\n", agent->desideredAngle.GetValue());
    //if( agent->desideredAngle < 0 )
     // agent->desideredAngle = CRadians::TWO_PI + agent->desideredAngle;
//    agent->desideredAngle = CRadians::ZERO;
    agent->desideredSpeed = 0;
    agent->desideredVelocity = CVector2(0, 0);

  }
  else  if (m_state == STATE_ARRIVED_AT_TARGET) 
  {
    agent->desideredAngle = CRadians::ZERO;
    agent->desideredSpeed = 0;
    agent->desideredVelocity = CVector2(0, 0);
  } 
  else 
  {
    updateDesideredVelocity();
  }
  agent->updateVelocity();
  updateVelocity();
}

  void 
RVONavClient::updateDesideredVelocity() 
{
  agent->targetPosition = m_targetPosition;
  agent->updateDesideredVelocity();
  //DEBUGNAV("=> desidered speed %.2f, desidered angle %.2f \n", 
	   //agent->desideredSpeed, 
	   //agent->desideredAngle.GetValue());
}

  void 
RVONavClient::updateVelocity() 
{
  //DEBUGNAV("%p: target wheel speed (%.3f %.3f) r %.3f\n", 
	 //agent, 
	 //agent->leftWheelTargetSpeed, 
	 //agent->rightWheelTargetSpeed, 
	 //agent->radius);
  m_pcWheels->SetLinearVelocity(
    100 * agent->leftWheelTargetSpeed / ODOMETRY_CORRECTION_FACTOR_LEFT, 
    100 * agent->rightWheelTargetSpeed / ODOMETRY_CORRECTION_FACTOR_RIGHT);
}

  void 
RVONavClient::updateVelocityProximitySensors() 
{
  /* Get readings from proximity sensor */
  const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
  /* Sum them together */
  CVector2 cAccumulator;
  for (size_t i = 0; i < tProxReads.size(); ++i) 
  {
    cAccumulator += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
  }
  cAccumulator /= tProxReads.size();
  /* If the angle of the vector is small enough and the closest obstacle is far enough,
     continue going straight, otherwise curve a little */
  CRadians cAngle = cAccumulator.Angle();
  if (m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cAngle) 
      && cAccumulator.Length() < m_fDelta) 
  {
    /// Go straight 
    //m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
  } 
  else 
  {
    /* Turn, depending on the sign of the angle */
    if (cAngle.GetValue() > 0.0f) {
      m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0f);
    } else {
      m_pcWheels->SetLinearVelocity(0.0f, m_fWheelVelocity);
    }
  }
  /* Set LED colors */
  m_ledsActuator->SetAllColors(CColor::BLACK);
  cAngle.UnsignedNormalize();
  UInt32 unIndex = static_cast<UInt32>(cAngle * 12.0f / CRadians::TWO_PI);
  m_ledsActuator->SetSingleColor(unIndex, CColor::RED);
}


std::string
RVONavClient::getTimeStr()
{
#ifndef FOOTBOT_LQL_SIM
  char buffer [80];
  timeval curTime;
  gettimeofday(&curTime, NULL);
  int milli = curTime.tv_usec / 1000;
  strftime(buffer, 80, "%Y-%m-%d %H:%M:%S", localtime(&curTime.tv_sec));
  char currentTime[84] = "";
  sprintf(currentTime, "%s:%d", buffer, milli);
  std::string ctime_str(currentTime);
  return ctime_str;
#else
  return "mytime";
#endif
}


/// returns time in milliseconds
  UInt64 
RVONavClient::getTime()
{
#ifndef FOOTBOT_LQL_SIM
  struct timeval timestamp;
  gettimeofday(&timestamp, NULL);

  UInt64 ms1 = (UInt64) timestamp.tv_sec;
  ms1*=1000;

  UInt64 ms2 = (UInt64) timestamp.tv_usec;
  ms2/=1000;

  return (ms1+ms2);
#else
  return m_Steps * CPhysicsEngine::GetSimulationClockTick() * 1000;
#endif
}



