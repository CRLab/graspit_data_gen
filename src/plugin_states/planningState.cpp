#include "plugin_states/planningState.h"
#include "include/EGPlanner/searchState.h"
#include "EGPlanner/simAnnPlanner.h"
#include "graspGenerationPlugin.h"
#include "body.h"

PlanningState::PlanningState(GraspGenerationPlugin *p): PluginState(),
    plugin(p),
    mHandObjectState(NULL),
    plannerStarted(false),
    maxSteps(70000)
{

}

int PlanningState::mainLoop()
{
    if(!plannerStarted)
    {
        plannerStarted = true;
        startPlanner();
    }
    else
    {
        stepPlanner();
    }

    return 1;
}


void PlanningState::startPlanner()
{
    mHandObjectState = new GraspPlanningState(plugin->mHand);
    mHandObjectState->setObject(plugin->mObject);
    mHandObjectState->setPositionType(SPACE_AXIS_ANGLE);
    mHandObjectState->setRefTran(plugin->mObject->getTran());
    mHandObjectState->reset();

    plugin->mPlanner = new SimAnnPlanner(plugin->mHand);
    ((SimAnnPlanner*)plugin->mPlanner)->setModelState(mHandObjectState);

    plugin->mPlanner->setEnergyType(ENERGY_CONTACT_QUALITY);
    plugin->mPlanner->setContactType(CONTACT_PRESET);
    plugin->mPlanner->setMaxSteps(maxSteps);

    plugin->mPlanner->resetPlanner();
    plugin->mPlanner->startPlanner();
}


void PlanningState::stepPlanner()
{
    if ( plugin->mPlanner->getCurrentStep() >= maxSteps)
    {
        plugin->mPlanner->stopPlanner();
        finished=true;
    }
}
