#ifndef GRASPGENERATOR_H
#define GRASPGENERATOR_H

#include <map>
#include <QObject>
#include <QtGui>

//GraspIt! includes
#include <include/plugin.h>
#include "mongo/client/dbclient.h" // for the driver


//Qt headers
#include <QJsonObject.h>

class EGPlanner;
class GraspPlanningState;
class GraspableBody;
class Hand;
class SensorReading;
class World;



//! Main class, combining a ROS node with a GraspIt! interface
/*! Note that this class inherits from GraspIt's Plugin class and implements the necessary functions to serve
  as a GraspIt plugin. See include/plugin.h in the GraspIt code for the base class.
  Provides a number of ROS services that directly operate on the GraspIt world, such as loading objects or
  obstacles, simulating 3D scans of objects, etc.
  In particular, note that this class uses the mainLoop() function to perform the ROS event management calls.
*/
class GraspGenerationPlugin : public QObject, public Plugin
{

    Q_OBJECT

public:
  //! Inits ROS, but (for now) without passing any arguments
  GraspGenerationPlugin();
  //! Deletes the node handle and the db manager
  ~GraspGenerationPlugin();
  //! Creates the node handles, advertises services, connects to the database
  virtual int init(int argc, char **argv);
  //! Simply calls ros::spinOnce() to process the ROS event loop
  virtual int mainLoop();

protected:
  void startPlanner();
  void uploadResults();
  void stepPlanner();
  mongo::BSONObj toMongoGrasp(GraspPlanningState *gps, QString energyType);
  mongo::BSONObj toMongoTactileGrasp(GraspPlanningState *gps, QString energyType);
  mongo::BSONObj toMongoSensorReading(SensorReading &sensorReading);

  void toMongoGraspBuilder(GraspPlanningState *gps, QString energyType, mongo::BSONObjBuilder *grasp);

  bool getSimHandSensors(World * w, std::vector<SensorReading> &sensorReadings);
private:

  EGPlanner *mPlanner;
  GraspPlanningState *mHandObjectState;
  GraspableBody *mObject;
  Hand *mHand;
  QJsonObject modelJson;
  bool plannerStarted;
  bool plannerFinished;
  bool evaluatingGrasps;
  int myInteger;
  QString dbName;
//  std::string dbName;
//  QString myQStr;
  bool render_it;
  //mongo::DBClientConnection *c;
  mongo::DBClientBase *c;
//  mongo::ConnectionString cs;

};


#endif // GRASPGENERATOR_H
