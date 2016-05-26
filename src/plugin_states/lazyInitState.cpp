#include "plugin_states/lazyInitState.h"

#include "graspGenerationPlugin.h"

#include "include/dbModelLoader.h"
#include "include/body.h"
#include "include/world.h"
#include "include/grasp.h"
#include "robot.h"

LazyInitState::LazyInitState(GraspGenerationPlugin *p): PluginState(),
  plugin(p)
{

}

int LazyInitState::mainLoop()
{

//     DbModelLoader loader;
//     plugin->modelJson = loader.loadRandomModel();

    plugin->modelJson = plugin->loadModel();

    plugin->mObject = graspItGUI->getMainWorld()->getGB(0);
    plugin->mObject->setMaterial(5);//rubber

    plugin->mHand = graspItGUI->getMainWorld()->getCurrentHand();
    plugin->mHand->getGrasp()->setObjectNoUpdate(plugin->mObject);
    plugin->mHand->getGrasp()->setGravity(false);

    finished = true;

    return 1;
}
