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
class PluginState;


class GraspGenerationPlugin : public QObject, public Plugin
{

    Q_OBJECT

public:

  GraspGenerationPlugin();
  ~GraspGenerationPlugin();
  virtual int init(int argc, char **argv);
  virtual int mainLoop();

  //shared variables that are accessed by the different states of the plugin.
  EGPlanner *mPlanner;
  GraspableBody *mObject;
  Hand *mHand;
  QJsonObject modelJson;
  QString dbName;
  mongo::DBClientBase *c;
  bool render_it;

protected:

  void evaluateGrasp(int result_idx);
  bool getSimHandSensors(World * w, std::vector<SensorReading> &sensorReadings);
  void fillGraspPlanningState(GraspPlanningState &gps);

private:

  std::vector<PluginState*> states;

};


#endif // GRASPGENERATOR_H
