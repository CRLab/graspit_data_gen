#ifndef MONGO_UTILS_H
#define MONGO_UTILS_H\

#include <QJsonObject.h>

#include "mongo/client/dbclient.h" // for the driver

class GraspPlanningState;
class SensorReading;
class transf;

namespace MongoUtils
{

mongo::BSONObj toBSON(GraspPlanningState *gps);
mongo::BSONObj toBSON(transf &frame);
mongo::BSONObj toBSON(SensorReading &sensorReading);
mongo::BSONArray toBSON(std::vector<SensorReading> &sensorReadings);
mongo::BSONObj toBSON(QJsonObject modelJson);

}

#endif
