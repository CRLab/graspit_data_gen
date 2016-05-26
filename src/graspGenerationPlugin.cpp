#include "graspGenerationPlugin.h"

#include "QJsonObject.h"
#include <cmath>
#include <fstream>

#include <include/world.h>
#include <include/body.h>
#include <include/robot.h>
#include <include/graspitGUI.h>
#include <include/ivmgr.h>

// qjson: might need to remove some of them
#include "qjson4/QJsonArray.h"
#include "qjson4/QJsonDocument.h"
#include "qjson4/QJsonObject.h"
#include "qjson4/QJsonParseError.h"
#include "include/dbModelLoader.h"

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


#include <cstdlib>
#include <iostream>

#include "mongo/client/dbclient.h" // for the driver
#include "mongoUtils.h"
#include "sensorReading.h"
#include "plugin_states/pluginState.h"
#include "include/robot.h"

#include "plugin_states/lazyInitState.h"
#include "plugin_states/planningState.h"
#include "plugin_states/evaluationState.h"

using mongo::BSONArray;
using mongo::BSONArrayBuilder;
using mongo::BSONObj;
using mongo::BSONObjBuilder;
using mongo::BSONElement;
using namespace mongo;


GraspGenerationPlugin::GraspGenerationPlugin()
{

}

GraspGenerationPlugin::~GraspGenerationPlugin()
{
}        


int GraspGenerationPlugin::init(int argc, char **argv)
{
    std::cout << "Starting GraspGenerationPlugin: " << std::endl ;
    std::cout << "Connecting to Mongo..." << std::endl ;

    mongo::client::GlobalInstance instance;
    if (!instance.initialized()) {
        std::cout << "failed to initialize the client driver: " << instance.status() << std::endl;
        return EXIT_FAILURE;
    }
    try {
        std::string uri = QString(getenv("MONGO_URL")).toStdString();
        if(uri == "") {
            std::cerr << "MONGO_URL env not found" << std::endl;
            return EXIT_FAILURE;
        }

        //"mongodb://tim:ilovetim@ds023418.mlab.com:23418/goparse";
        std::string errmsg;

        ConnectionString cs = ConnectionString::parse(uri, errmsg);

        if (!cs.isValid()){
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
    } catch( const mongo::DBException &e ) {
        std::cout << "caught " << e.what() << std::endl;
    }

    cmdline::parser *parser = new cmdline::parser();
    parser->add<std::string>("dbname", 'c', "dbname",  false);
    parser->add<bool>("render", 'l', "render", false);

    parser->parse(argc, argv);

    if (parser->exist("render")) {
        render_it = parser->get<bool>("render");
    }
    else{
        render_it = false;
    }

    std::cout << "Finished Init..." << std::endl;

    states.push_back( new LazyInitState(this));
    states.push_back( new PlanningState(this));
    states.push_back( new EvaluationState(this));

    return 0;
}


int GraspGenerationPlugin::mainLoop()
{
    //if there are no more states, then we are finished
    if (states.size() == 0)
    {
        assert(false);
    }

    PluginState * current_state = states.at(0);
    current_state->mainLoop();

    if (current_state->isFinished()){
        states.erase(states.begin());
    }

    return 0;
}



// This is a custom helper to load hardcoded model while not breaking the dbModelLoader interface
QJsonObject GraspGenerationPlugin::loadModel() {


    QJsonObject model;
    QString hand("BarrettBH8_280_Tactile");
    model["url"] = "http://borneo.cs.columbia.edu/modelnet/vision.cs.princeton.edu/projects/2014/ModelNet/data/bottle/bottle_000000161/bottle_000000161.off";
    model["name"] = "bottle-2HcMVYKKB";
    model["material"] = "rubber";
    model["dimension"] = 100;


//    QString url = modelJson["url"].toString();
//    QString modelName = modelJson["name"].toString();
//    QString material = modelJson["material"].toString();
//    double dimension = modelJson["dimension"].toDouble();

//    return model.obj();
    DbModelLoader loader;
    QString robotPath = QString(getenv("GRASPIT")) + QString("/models/robots/") + hand + QString("/") + hand + QString(".xml");


//    std::cout << url.toStdString().c_str() << std::endl;

    graspItGUI->getMainWorld()->importRobot(robotPath.toStdString().c_str());

    loader.loadModelFromUrl(model["url"].toString(), model["name"].toString(), model["material"].toString(), model["dimension"].toDouble());

    return model;
 }
