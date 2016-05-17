#include "mongoUtils.h"
#include "modelInfo.h"
#include "sensorReading.h"
#include "robot.h"
#include "EGPlanner/searchState.h"

using mongo::BSONArray;
using mongo::BSONArrayBuilder;
using mongo::BSONObj;
using mongo::BSONObjBuilder;
using mongo::BSONElement;
using namespace mongo;

namespace MongoUtils
{

    mongo::BSONObj toBSON(QJsonObject modelJson)
    {
        BSONObjBuilder model;

        QString url = modelJson["url"].toString();
        QString modelName = modelJson["name"].toString();
        QString material = modelJson["material"].toString();
        double dimension = modelJson["dimension"].toDouble();

        model.append("name", modelName.toStdString());
        model.append("url", url.toStdString());
        model.append("material", material.toStdString());
        model.append("dimension", dimension);

        return model.obj();
    }


    mongo::BSONObj toBSON(GraspPlanningState *gps)
    {
        BSONObjBuilder grasp;
        BSONObjBuilder pose;

        BSONObjBuilder energy;
        BSONArrayBuilder translation;
        BSONArrayBuilder rotation;
        BSONArrayBuilder dof;

        Hand *hand = gps->getHand();
        GraspableBody *body = gps->getObject();

        double dofVals [hand->getNumDOF()];
        hand->getDOFVals(dofVals);

        transf hand_pose = hand->getPalm()->getTran();

        for(int dof_idx = 0; dof_idx < hand->getNumDOF(); dof_idx ++)
        {
            dof.append(dofVals[dof_idx]);
        }

        energy.append("ENERGY_CONTACT_QUALITY",  gps->getEnergy());
        energy.append("Volume", gps->getVolume());
        energy.append("Epsilon", gps->getEpsilonQuality());

        translation.append(hand_pose.translation().x())
                .append(hand_pose.translation().y())
                .append(hand_pose.translation().z());

        rotation.append(hand_pose.rotation().w)
                .append(hand_pose.rotation().x)
                .append(hand_pose.rotation().y)
                .append(hand_pose.rotation().z);

        pose.append("translation", translation.arr());
        pose.append("rotation", rotation.arr());

        grasp.append("hand", hand->getDBName().toStdString());
        grasp.append("energy", energy.obj());
        grasp.appendArray("dof", dof.arr());
        grasp.append("pose", pose.obj());

        return grasp.obj();
    }

    mongo::BSONArray toBSON(std::vector<SensorReading> &sensorReadings)
    {
        BSONArrayBuilder sensorReadingsArr;
        for(unsigned int idx = 0; idx < sensorReadings.size(); idx ++)
        {
            BSONObj  reading = toBSON(sensorReadings.at(idx));
            sensorReadingsArr.append(reading);
        }
        return sensorReadingsArr.arr();
    }


    mongo::BSONObj toBSON(SensorReading &sensorReading)
    {
        BSONObjBuilder readingBSON;
        BSONArrayBuilder translation;
        BSONArrayBuilder orientation;

        translation.append(sensorReading.translation.x())
                   .append(sensorReading.translation.y())
                   .append(sensorReading.translation.z());

        orientation.append(sensorReading.orientation.w)
                   .append(sensorReading.orientation.x)
                   .append(sensorReading.orientation.y)
                   .append(sensorReading.orientation.z);

        readingBSON.append("force", sensorReading.force);
        readingBSON.append("translation", translation.obj());
        readingBSON.append("orientation", orientation.obj());

        return readingBSON.obj();
    }

    mongo::BSONObj toBSON(transf &frame)
    {
        BSONObjBuilder bsonframe;
        BSONArrayBuilder translation;
        BSONArrayBuilder orientation;

        translation.append(frame.translation().x())
                   .append(frame.translation().y())
                   .append(frame.translation().z());

        orientation.append(frame.rotation().w)
                   .append(frame.rotation().x)
                   .append(frame.rotation().y)
                   .append(frame.rotation().z);

        bsonframe.append("translation", translation.obj());
        bsonframe.append("orientation", orientation.obj());

        return bsonframe.obj();
    }
}
