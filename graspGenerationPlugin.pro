
#to build as a make file in windows, change vclib to lib
win32{
TEMPLATE	= vclib
}else{
TEMPLATE = lib
}

LANGUAGE	= C++
CONFIG += qt dylib c++11
QMAKE_CXXFLAGS += -g -std=c++11
QT += qt3support
DESTDIR = lib
MOC_DIR = build
OBJECTS_DIR = build

!exists($(GRASPIT)) {
   error("GRASPIT environment variable not set")
}

!exists($(MONGO_CXX_PATH)) {
   error("MONGO_CXX_PATH environment variable not set")
    #/home/jvarley/graspit_data_gen/mongo-cxx-driver/
}

INCLUDEPATH += include include/plugin_states $(GRASPIT) $(GRASPIT)/qjson4 $(GRASPIT)/include $(GRASPIT)/cmdline

# Mongo Driver headers
INCLUDEPATH += $(MONGO_CXX_PATH)/build/install/include


DEPENDPATH += $(GRASPIT)/src src src/plugin_states

HEADERS += $(GRASPIT)/include/plugin.h \
    include/graspGenerationPlugin.h \
    include/mongoUtils.h \
    include/modelInfo.h \
    include/sensorReading.h \
    include/plugin_states/pluginState.h \
    include/plugin_states/planningState.h \
    include/plugin_states/evaluationState.h \
    include/plugin_states/lazyInitState.h

SOURCES += \
    src/main.cpp \
    src/graspGenerationPlugin.cpp\
    src/mongoUtils.cpp \
    src/modelInfo.cpp \
    src/plugin_states/pluginState.cpp \
    src/plugin_states/planningState.cpp \
    src/plugin_states/evaluationState.cpp \
    src/plugin_states/lazyInitState.cpp

# Mongo Driver shared lib
LIBS += -L$(MONGO_CXX_PATH)/build/install/lib -lmongoclient
