#include "constants.h"
#include <string>

struct ScreenSetup{

    int prevAuto(int autoNum) {
        return --autoNum;
    }
    
    bool selectAuto() {
        return false;
    }
    
    int nextAuto(int autoNum) {
        return ++autoNum;
    }
    
    int nextScreen(int currentScreen) {
        return (currentScreen + 1) % 3;
    }
    
    int prevScreen(int currentScreen) {
        return (currentScreen + 2) % 3;
    }
};

struct ControlSetup {

    bool toggle;
    bool inverseOutput;

    int state = 0;
    int outputType;
    
    ControlSetup(int outputType,
                bool toggle = false,
                bool inverseOutput = false
                ):
                outputType(outputType),
                toggle(toggle),
                inverseOutput(inverseOutput) 
                {}

    int use(bool condition) {
        if (toggle && condition) state = !state;
        else if (condition) state = true;

        if (outputType == MOTOR && state && inverseOutput) return -12000;
        else if (outputType == MOTOR && state) return 12000;

        if (outputType == DIGITAL) return state;

        return 0;
    }
};

struct RecordingSetup {
  	int value;
  	int lastValue;
  	int controlType;
  
  	std::string name;
  	
  	FILE* fileName;
  
  	RecordingSetup(FILE* fileName,
  				int controlType)
  				:
  				fileName(fileName),
  				controlType(controlType)
  				{}

  	void update(const std::string& name, int value, int lastValue) {
        if (value != lastValue && controlType == MOTOR) fprintf(fileName, "%s.move_voltage(%d);", name.c_str(), value);
        else if (value != lastValue && controlType == DIGITAL) fprintf(fileName, "%s.set_value(%d);", name.c_str(), value);
        else if (value != lastValue) fprintf(fileName, "%s=%d;", name.c_str(), value);
    }
};