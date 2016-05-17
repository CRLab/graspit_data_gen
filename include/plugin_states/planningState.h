#ifndef PLANNING_STATE_H
#define PLANNING_STATE_H

#include "plugin_states/pluginState.h"

class EGPlanner;
class GraspPlanningState;
class GraspGenerationPlugin;

class PlanningState : public PluginState
{
public:
     PlanningState(GraspGenerationPlugin *p);
    virtual int mainLoop();


protected:
    GraspGenerationPlugin *plugin;
    GraspPlanningState *mHandObjectState;
    bool plannerStarted;
    int maxSteps;

    void startPlanner();
    void stepPlanner();

};


#endif
