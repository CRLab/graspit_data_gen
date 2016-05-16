#include "graspGenerationPlugin.h"

#include "QJsonObject.h"
#include <cmath>
#include <fstream>

#include <include/world.h>
#include <include/body.h>
#include <include/robot.h>
#include <include/graspitGUI.h>
#include <include/ivmgr.h>

#include <include/EGPlanner/egPlanner.h>
#include <include/EGPlanner/simAnnPlanner.h>
#include <include/EGPlanner/onLinePlanner.h>
#include <include/EGPlanner/guidedPlanner.h>
#include <include/EGPlanner/searchState.h>
#include <include/EGPlanner/energy/searchEnergy.h>

#include <include/SensorInterface.h>
#include <include/debug.h>

#include <include/grasp.h>
#include "quality.h"
#include <include/triangle.h>

#include <cmdline/cmdline.h>
#include "include/dbModelLoader.h"

#include <cstdlib>
#include <iostream>

#include "mongo/client/dbclient.h" // for the driver

using mongo::BSONArray;
using mongo::BSONArrayBuilder;
using mongo::BSONObj;
using mongo::BSONObjBuilder;
using mongo::BSONElement;
using namespace mongo;

struct SensorReading
{
    vec3 translation;
    Quaternion orientation;
    double force;
    // {"location": [x, y, z], "orientation"}
    void printMe() const {
        DBGA(" force: " << force <<
             " {" << translation.x() << ", " << translation.y() << ", " << translation.z() << "} " <<
             " [" << orientation.w << ", " << orientation.x << ", " << orientation.y << ", " << orientation.z << "]");
    }
};

GraspGenerationPlugin::GraspGenerationPlugin() :
    mPlanner(NULL),
    plannerStarted(false),
    plannerFinished(false),
    evaluatingGrasps(false),
    currently_uploading_result_idx(0)
{

}

GraspGenerationPlugin::~GraspGenerationPlugin()
{
}        


int GraspGenerationPlugin::init(int argc, char **argv)
{
    std::cout << "Starting GraspGenerationPlugin: " << std::endl ;
    std::cout << "Connecting to Mongo..." << std::endl ;
    std::cout << "My Integer" << myInteger << std::endl;

    mongo::client::GlobalInstance instance;
    if (!instance.initialized()) {
        std::cout << "failed to initialize the client driver: " << instance.status() << std::endl;
        return EXIT_FAILURE;
    }
    try {


        std::string uri = QString(getenv("MONGO_URL")).toStdString();
        if(uri == "") {

            std::cerr << "MONGO_URL env not found" << std::endl;
            return 1;
        }

//                "mongodb://tim:ilovetim@ds023418.mlab.com:23418/goparse";
        std::string errmsg;

        ConnectionString cs = ConnectionString::parse(uri, errmsg);

        if (!cs.isValid()) {
            std::cout << "Error parsing connection string " << uri << ": " << errmsg << std::endl;
            return EXIT_FAILURE;
        }

        c = cs.connect(errmsg);
        if (!c) {
            std::cout << "couldn't connect : " << errmsg << std::endl;
            return EXIT_FAILURE;
        }

        dbName = QString::fromStdString(cs.getDatabase());

        std::cout << "Connected to database: "<< dbName.toStdString().c_str() << std::endl;


        std::cout << "connected ok to mongodb" << std::endl;
    } catch( const mongo::DBException &e ) {
        std::cout << "caught " << e.what() << std::endl;
    }


    std::cout << "Parsing Args..." << std::endl;
    cmdline::parser *parser = new cmdline::parser();

    parser->add<std::string>("dbname", 'c', "dbname",  false);
    parser->add<bool>("render", 'l', "render", false);

    parser->parse(argc, argv);

    if (parser->exist("render"))
    {
        render_it = parser->get<bool>("render");

    }
    else
    {
        render_it = false;
    }

//    dbName = QString::fromStdString(parser->get<std::string>("dbname"));


    std::cout << "Args are: " << std::endl;
    std::cout << "render: " << render_it << "\n" ;
//    std::cout << "dbName: " << dbName.toStdString().c_str() << "\n" ;

    std::cout << "Finished Init..." << std::endl;

    return 0;
}

//This loop is called over and over again. We do 3 different things
// 1) First step: start the planner
// 2) Middle steps: step the planner
// 3) Last step, save the grasps
int GraspGenerationPlugin::mainLoop()
{
    //start planner
    if (!plannerStarted)
    {
        startPlanner();
    }
    //let planner run.
    else if( (plannerStarted) && !plannerFinished )
    {
        stepPlanner();
    }
    //save grasps
    else if(plannerStarted && plannerFinished && (!evaluatingGrasps))
    {
        uploadResult(currently_uploading_result_idx);
        currently_uploading_result_idx++;
    }

  return 0;
}

void GraspGenerationPlugin::startPlanner()
{
    std::cout << "Starting Planner\n" ;

    //TODO
    //here we need to get the hand and object from the cloud. rather than locally
    QString modelUrl = "http://borneo.cs.columbia.edu/modelnet/vision.cs.princeton.edu/projects/2014/ModelNet/data/pyramid/pyramid_000001463/pyramid_000001463.off";
    DbModelLoader loader;
//    loader.loadModelFromUrl(modelUrl, QString(""), QString("rubber"));

    modelJson = loader.loadRandomModel();

    mObject = graspItGUI->getMainWorld()->getGB(0);
    mObject->setMaterial(5);//rubber

    mHand = graspItGUI->getMainWorld()->getCurrentHand();
    mHand->getGrasp()->setObjectNoUpdate(mObject);
    mHand->getGrasp()->setGravity(false);

    mHandObjectState = new GraspPlanningState(mHand);
    mHandObjectState->setObject(mObject);
    mHandObjectState->setPositionType(SPACE_AXIS_ANGLE);
    mHandObjectState->setRefTran(mObject->getTran());
    mHandObjectState->reset();

    mPlanner = new SimAnnPlanner(mHand);
    ((SimAnnPlanner*)mPlanner)->setModelState(mHandObjectState);

    mPlanner->setEnergyType(ENERGY_CONTACT_QUALITY);
    mPlanner->setContactType(CONTACT_PRESET);
    mPlanner->setMaxSteps(70000);

    mPlanner->resetPlanner();
    mPlanner->startPlanner();

    plannerStarted = true;
}

void GraspGenerationPlugin::stepPlanner()
{
    if ( mPlanner->getCurrentStep() >= 70000)
    {
        mPlanner->stopPlanner();
        plannerFinished=true;
    }
}


BSONObj GraspGenerationPlugin::buildParentGrasp(int result_idx)
{
    SearchEnergy *mEnergyCalculator = SearchEnergy::getSearchEnergy(ENERGY_CONTACT_QUALITY);
    mEnergyCalculator->setContactType(CONTACT_PRESET);


    if (result_idx == mPlanner->getListSize())
    {
        assert(false);
    }

    std::cout << "Found " << mPlanner->getListSize() << " Grasps. " << std::endl;

    GraspPlanningState gps = mPlanner->getGrasp(result_idx);
    gps.execute(mHand);
    mHand->autoGrasp(render_it, 1.0, false);

    bool is_legal;
    double new_planned_energy;
    mEnergyCalculator->analyzeCurrentPosture(mHand,graspItGUI->getMainWorld()->getGB(0),is_legal,new_planned_energy,false );
    gps.setEnergy(new_planned_energy);
    gps.saveCurrentHandState();

    QualEpsilon mQualEpsilon(mHand->getGrasp(), QString("Grasp_recorder_qm"), "L1 Norm");
    double epsilonQuality = mQualEpsilon.evaluate();

    QualVolume mQualVolume(mHand->getGrasp(), QString("Grasp_recorder_qm"), "L1 Norm");
    double volumeQuality = mQualVolume.evaluate();

    gps.setVolume(volumeQuality);
    gps.setEpsilonQuality(epsilonQuality);

    graspItGUI->getIVmgr()->getViewer()->render();

    std::vector<SensorReading> sensorReadings;
    getSimHandSensors(graspItGUI->getMainWorld(), sensorReadings);
    std::cout << "num sensor readings: " << sensorReadings.size() << std::endl;

    graspItGUI->getIVmgr()->getViewer()->render();


    return toMongoTactileGrasp(&gps, QString("ENERGY_CONTACT_QUALITY"));

}

std::vector<BSONObj> GraspGenerationPlugin::buildChildren(BSONObj parent)
{
    std::vector<BSONObj> children;
    //    //peturb and save
    //    transf perturbation = rotXYZ(0.25, 0, 0);
    ////    transf perturbation = rotXYZ(0, 0.25, 0);
    ////    transf perturbation = rotXYZ(0, 0, 0.25);

    ////    transf perturbation = rotXYZ(-0.25, 0, 0);
    ////    transf perturbation = rotXYZ(0, -0.25, 0,0);
    ////    transf perturbation = rotXYZ(0, 0, -0.25);

    //    //Disable collisions between the ground truth object and the hand
    //      graspItGUI->getMainWorld()->toggleAllCollisions(false);
    //      sleep(1.0);

    //      graspItGUI->getIVmgr()->getViewer()->render();
    //      transf currentTran = mHand->getTran();
    //      transf newTran = perturbation* currentTran  ;

    //      mHand->setTran(newTran);

    //      mHand->autoGrasp(true, -1.0, false);
    //      mHand->approachToContact(-200);
    //      graspItGUI->getIVmgr()->getViewer()->render();

    //      graspItGUI->getMainWorld()->toggleAllCollisions(true);
    //      sleep(1);

    //      mHand->approachToContact(200);
    //      mHand->autoGrasp(true, 1.0, false);
    //      graspItGUI->getIVmgr()->getViewer()->render();
    return children;
}

void GraspGenerationPlugin::uploadResult(int result_idx)
{
    BSONObj p = buildParentGrasp(result_idx);

    //generate lots of children
    std::vector<BSONObj> children = buildChildren(p);

    //attach children to parent
    //for loop

    //upload children
    //for loop

    // parent died ;(
    //upload parent
    std::string mongoCollName = (dbName + QString(".grasps")).toStdString();
    std::cout <<"Uploading to Mongo Coll: " << mongoCollName << std::endl;
    c->insert(mongoCollName, p);
}

mongo::BSONObj GraspGenerationPlugin::toMongoGrasp(GraspPlanningState *gps, QString energyType)
{
    BSONObjBuilder grasp;
    toMongoGraspBuilder(gps, energyType, &grasp);

    return grasp.obj();
}

mongo::BSONObj GraspGenerationPlugin::toMongoSensorReading(SensorReading &sensorReading) {


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

bool GraspGenerationPlugin::getSimHandSensors(World * w, std::vector<SensorReading> &sensorReadings) {
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

mongo::BSONObj GraspGenerationPlugin::toMongoTactileGrasp(GraspPlanningState *gps, QString energyType,  std::vector<SensorReading> &sensorReadings) {
    BSONObjBuilder grasp;
    toMongoGraspBuilder(gps, energyType, &grasp);

    BSONArrayBuilder sensorReadingsBuilder;

    for(int i = 0; i < sensorReadings.size(); i++) {

        sensorReadingsBuilder.append(toMongoSensorReading(sensorReadings.at(i)));
//        std::cout << sensorReadingBSONObj << std::endl;
    }

    grasp.append("tactile_states", sensorReadingsBuilder.obj());
    //    grasps.append("neighbor_grasps")
    // key: neighbour_grasps, value: {"string_idx": "mongo_object_id", "string_idx2": "mongo_object_id2"} store the neighbour grasps ahead of time and retrieve their ids
    // key:tactile, value: bsonObj[64] which tactile sensor is activated, idx of array does matter
    return grasp.obj();

}

void GraspGenerationPlugin::toMongoGraspBuilder(GraspPlanningState *gps, QString energyType, mongo::BSONObjBuilder *grasp)
{

    BSONObjBuilder pose;
    BSONObjBuilder model;
    BSONObjBuilder energy;
    BSONArrayBuilder translation;
    BSONArrayBuilder rotation;
    BSONArrayBuilder dof;

    Hand *hand = gps->getHand();
    GraspableBody *body = gps->getObject();

    double dofVals [hand->getNumDOF()];
    hand->getDOFVals(dofVals);

    transf hand_pose = mHand->getPalm()->getTran();

    for(int dof_idx = 0; dof_idx < hand->getNumDOF(); dof_idx ++)
    {
        dof.append(dofVals[dof_idx]);
    }

    energy.append("ENERGY_CONTACT_QUALITY",  gps->getEnergy());
    energy.append("Volume", gps->getVolume());
    energy.append("Epsilon", gps->getEpsilonQuality());

    translation.append(hand_pose.translation().x()).append(hand_pose.translation().y()).append(hand_pose.translation().z());
    rotation.append(hand_pose.rotation().w).append(hand_pose.rotation().x).append(hand_pose.rotation().y).append(hand_pose.rotation().z);
    pose.append("translation", translation.arr());
    pose.append("rotation", rotation.arr());

    QString url = modelJson["url"].toString();
    QString modelName = modelJson["name"].toString();
    QString material = modelJson["material"].toString();
    double dimension = modelJson["dimension"].toDouble();

    model.append("name", modelName.toStdString());
    model.append("url", url.toStdString());
    model.append("material", material.toStdString());
    model.append("dimension", dimension);
    grasp->append("model", model.obj());
    grasp->append("hand", hand->getDBName().toStdString());
    grasp->append("energy", energy.obj());
    grasp->appendArray("dof", dof.arr());
    grasp->append("pose", pose.obj());
}


