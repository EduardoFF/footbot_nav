#include "footbot_navigator.h"

FootbotNavigator::FootbotNavigator() :
  RandomSeed(12345),
  m_Steps(0)
{
}


  void 
FootbotNavigator::Init(TConfigurationNode& t_node) 
{
  /// The first thing to do, set my ID
#ifdef FOOTBOT_LQL_SIM
  m_myID = 
    atoi(GetRobot().GetRobotId().substr(3).c_str());
#else
  m_myID = 
    atoi(GetRobot().GetRobotId().substr(7).c_str());
#endif
  printf("MyID %d\n", m_myID);

  /// Random
  GetNodeAttributeOrDefault(t_node, "RandomSeed", RandomSeed, RandomSeed);

  /// create the client and pass the configuration tree to it
  m_navClient = new RVONavClient(m_myID, GetRobot());
  m_navClient->init(t_node);

}


  void 
FootbotNavigator::ControlStep() 
{
  /* do whatever */
  m_navClient->update();
}


  void 
FootbotNavigator::Destroy() 
{
  DEBUG_CONTROLLER("FootbotNavigator::Destroy (  )\n");
}

/**************************************/

bool 
FootbotNavigator::IsControllerFinished() const 
{
  return false;
}

REGISTER_CONTROLLER(FootbotNavigator, "footbot_navigator")

