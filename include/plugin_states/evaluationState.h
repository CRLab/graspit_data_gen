#ifndef EVALUATION_STATE_H
#define EVALUATION_STATE_H

#include "plugin_states/pluginState.h"
#include "graspGenerationPlugin.h"
#include "body.h"
#include "EGPlanner/searchState.h"

class EvaluationState: public PluginState
{
public:
    EvaluationState(GraspGenerationPlugin *p);
    virtual int mainLoop();

protected:
    GraspGenerationPlugin *plugin;

    std::vector<transf> perturbations;

    void evaluateGrasp(int result_idx);
    bool getSimHandSensors(World * w, std::vector<SensorReading> &sensorReadings);
    void fillGraspPlanningState(GraspPlanningState &gps);

    int currently_uploading_result_idx;

};

#endif
