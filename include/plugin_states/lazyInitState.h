#ifndef LAZY_INIT_STATE_H
#define LAZY_INIT_STATE_H

#include "plugin_states/pluginState.h"

class GraspGenerationPlugin;

class LazyInitState: public PluginState
{
public:
    LazyInitState(GraspGenerationPlugin *p);
    virtual int mainLoop();

protected:
    GraspGenerationPlugin *plugin;
};


#endif
