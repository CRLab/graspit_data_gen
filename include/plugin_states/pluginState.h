#ifndef PLUGIN_STATE_H
#define PLUGIN_STATE_H

class PluginState
{

public:
    PluginState();
    virtual int mainLoop() = 0;
    virtual ~PluginState();

    bool isFinished();

protected:
    bool finished;
};


#endif
