#include "plugin_states/pluginState.h"


PluginState::PluginState():
    finished(false)
{

}


bool PluginState::isFinished()
{
    return finished;
}


PluginState::~PluginState()
{

}

