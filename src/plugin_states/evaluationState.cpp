#include "plugin_states/evaluationState.h"

#include "quality.h"
#include "mongoUtils.h"
#include "EGPlanner/energy/searchEnergy.h"
#include "sensorReading.h"
#include "include/world.h"
#include "include/robot.h"
#include "SensorInterface.h"
#include "graspitGUI.h"
#include "EGPlanner/egPlanner.h"
#include "ivmgr.h"

using namespace mongo;

EvaluationState::EvaluationState(GraspGenerationPlugin *p): PluginState(),
    plugin(p),
    currently_uploading_result_idx(0)
{
    perturbations.push_back(rotXYZ(0.25, 0, 0));
    perturbations.push_back(rotXYZ(0, 0.25, 0));
    perturbations.push_back(rotXYZ(0, 0, 0.25));
    perturbations.push_back(rotXYZ(-0.25, 0, 0));
    perturbations.push_back(rotXYZ(0, -0.25, 0));
    perturbations.push_back(rotXYZ(0, 0, -0.25));
    perturbations.push_back(translate_transf(vec3(0,0, -20)));
    perturbations.push_back(translate_transf(vec3(0,0, 20)));
}

int EvaluationState::mainLoop()
{
    evaluateGrasp(currently_uploading_result_idx);
    currently_uploading_result_idx++;

    //if we have uploaded all the grasps then it is time to exit
    if (currently_uploading_result_idx == plugin->mPlanner->getListSize())
    {
        finished = true;
    }
    return 1;
}


void EvaluationState::evaluateGrasp(int result_idx)
{
    GraspPlanningState gps = plugin->mPlanner->getGrasp(result_idx);
    gps.execute(plugin->mHand);
    plugin->mHand->autoGrasp(plugin->render_it, 1.0, false);

    //this calculates the epsilon, volume, and energy
    fillGraspPlanningState(gps);

    //this collects the sensor readings
    std::vector<SensorReading> sensorReadings;
    getSimHandSensors(graspItGUI->getMainWorld(), sensorReadings);

    std::string mongoCollectionName = (plugin->dbName + QString(".grasps")).toStdString();

    BSONArrayBuilder children;
    for(unsigned int j=0; j < perturbations.size(); j++)
    {
        transf perturbation = perturbations.at(j);

        std::cout << "return to original grasp" << std::endl;
        gps.execute(plugin->mHand);
        graspItGUI->getIVmgr()->getViewer()->render();

        std::cout << "Disable collisions" << std::endl;
        graspItGUI->getMainWorld()->toggleAllCollisions(false);

        transf currentTran = plugin->mHand->getTran();
        transf newTran = perturbation*currentTran  ;

        std::cout << "peturbing hand" << std::endl;
        plugin->mHand->setTran(newTran);

        std::cout << "opening hand" << std::endl;
        plugin->mHand->autoGrasp(true, -1.0, false);
        std::cout << "reverse approach to contact" << std::endl;
        plugin->mHand->approachToContact(-200);
        graspItGUI->getIVmgr()->getViewer()->render();

        std::cout << "reenable collisions" << std::endl;
        graspItGUI->getMainWorld()->toggleAllCollisions(true);

        std::cout << "approachToContact" << std::endl;
        plugin->mHand->approachToContact(200);
        std::cout << "autoGrasp" << std::endl;
        plugin->mHand->autoGrasp(true, 1.0, false);
        graspItGUI->getIVmgr()->getViewer()->render();

        std::cout << "computing energy" << std::endl;
        GraspPlanningState peturbedState(plugin->mHand);
        peturbedState.setObject(gps.getObject());
        std::cout << "saving current hand state" << std::endl;
        peturbedState.saveCurrentHandState();
        std::cout << "filling in grasp planning state" << std::endl;
        fillGraspPlanningState(peturbedState);

        std::cout << "getting tactile readings" << std::endl;
        std::vector<SensorReading> peturbedSensorReadings;
        getSimHandSensors(graspItGUI->getMainWorld(), peturbedSensorReadings);

        std::cout << "converting to bson" << std::endl;
        BSONObjBuilder peturbedPlanningFrame;
        peturbedPlanningFrame.append("grasp", MongoUtils::toBSON(&peturbedState));
        peturbedPlanningFrame.append("tactile", MongoUtils::toBSON(peturbedSensorReadings));
        peturbedPlanningFrame.append("model", MongoUtils::toBSON(plugin->modelJson));
        peturbedPlanningFrame.append("perturbation", MongoUtils::toBSON(perturbation));

        children.append(peturbedPlanningFrame.obj());

        std::cout << "uploading peturbed grasp" << std::endl;
        plugin->c->insert(mongoCollectionName, peturbedPlanningFrame.obj());
    }

    std::cout << "building grasp" << std::endl;
    BSONObjBuilder planningFrame;
    planningFrame.append("grasp", MongoUtils::toBSON(&gps));
    planningFrame.append("tactile", MongoUtils::toBSON(sensorReadings));
    planningFrame.append("model", MongoUtils::toBSON(plugin->modelJson));
    planningFrame.append("children", children.obj());

    std::cout << "uploading grasp" << std::endl;
    plugin->c->insert(mongoCollectionName, planningFrame.obj());
}




/**
 * @brief GraspGenerationPlugin::fillGraspPlanningState
 * @param gps
 *
 * When a grasp planning state is initially created, the volume and epsilon
 * are not filled in.  The energy has also changed since we have done an approach
 * and autoclose.
 */
void EvaluationState::fillGraspPlanningState(GraspPlanningState &gps)
{
    SearchEnergy *mEnergyCalculator = SearchEnergy::getSearchEnergy(ENERGY_CONTACT_QUALITY);
    mEnergyCalculator->setContactType(CONTACT_PRESET);

    bool is_legal;
    double new_planned_energy;
    mEnergyCalculator->analyzeCurrentPosture(gps.getHand(), gps.getObject(), is_legal, new_planned_energy, false );
    gps.setEnergy(new_planned_energy);
    gps.saveCurrentHandState();

    QualEpsilon mQualEpsilon(plugin->mHand->getGrasp(), QString("Grasp_recorder_qm_epsilon"), "L1 Norm");
    double epsilonQuality = mQualEpsilon.evaluate();

    QualVolume mQualVolume(plugin->mHand->getGrasp(), QString("Grasp_recorder_qm_volume"), "L1 Norm");
    double volumeQuality = mQualVolume.evaluate();

    gps.setVolume(volumeQuality);
    gps.setEpsilonQuality(epsilonQuality);
}

bool EvaluationState::getSimHandSensors(World * w, std::vector<SensorReading> &sensorReadings) {
    for(int sensorInd = 0; sensorInd < w->getNumSensors(); sensorInd ++) {
        QString str;
        QTextStream qts(&str, QIODevice::WriteOnly);

        w->getSensor(sensorInd)->updateSensorModel();
        w->getSensor(sensorInd)->outputSensorReadings(qts);

        char sensorType[20];
        //bounding box of the sensor
        double lx, ly, lz, rx, ry, rz;
        //force and torque
        double fx, fy, fz, tx, ty, tz;
        sscanf(str.toStdString().c_str(), "%s %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf", sensorType,
            &lx, &ly, &lz, &rx, &ry, &rz,
            &fx, &fy, &fz, &tx, &ty, &tz);

        //get the contact orientation
        transf sensorInWorld = w->getSensor(sensorInd)->getSensorTran(); // considered as world-to-sensor transform
        transf handInWorld = w->getCurrentHand()->getTran(); // considered as world-to-hand transform
        transf sensorInHand = sensorInWorld * handInWorld.inverse(); // considered as hand-to-sensor = hand-to-world * world-to-sensor

        SensorReading sr;
        sr.force = fz;
        sr.translation = sensorInHand.translation();
        sr.orientation = sensorInHand.rotation();
        sensorReadings.push_back(sr);
    }

    return true;
}
