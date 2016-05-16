
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

INCLUDEPATH += $(GRASPIT) $(GRASPIT)/qjson4 $(GRASPIT)/include $(GRASPIT)/cmdline

# Mongo Driver headers
INCLUDEPATH += $(MONGO_CXX_PATH)/build/install/include


DEPENDPATH += $(GRASPIT)/src 

HEADERS += $(GRASPIT)/include/plugin.h \
    graspGenerationPlugin.h

SOURCES += \
    main.cpp \
    graspGenerationPlugin.cpp

# Mongo Driver shared lib
LIBS += -L$(MONGO_CXX_PATH)/build/install/lib -lmongoclient
