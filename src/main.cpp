#include "main.h"
#include "lemlib/chassis/trackingWheel.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/pid.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"
#include "pros/distance.hpp"
#include "lemlib/timer.hpp"
#include "pros/optical.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include "pros/misc.hpp"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/misc.h"

const int RED = 1;
const int BLUE = -1;
const int MOTOR = 1;
const int DIGITAL = 0;
const int VAR = 2;
const int IDLE = 2000;
const int LOAD = 4800;
const int SCORE = 16000;
const int FAR = 22800;
const bool TESTMODE = false;

int leftY;
int rightX;
int autoNum = 0;
int allyColor = 0;
int spintake = 0;
int ladybrown_target = 0;
int lastLeftY = 0;
int lastRightX = 0;
int lastFirstStageVoltage = 0;
int lastSecondStageVoltage = 0;

bool lastLeftDoink = false;
bool lastRightDoink = false;
bool lastClamp = false;
bool lastIntakeLift = false;
bool selecting = true;
bool loadState = false;
bool scoreState = false;
bool autoRunning = false;
bool recording = false;
bool lastLoadState = false;
bool lastScoreState = false;

float ladybrownErr = 0;
float lastWrite = 0;
float colorReading = 0;

FILE* recordings = nullptr;

pros::Controller ctrl(pros::E_CONTROLLER_MASTER);

pros::Motor firstStageIntake(9, pros::MotorGearset::green);
pros::Motor secondStageIntake(1, pros::MotorGearset::blue);

pros::Motor ladybrown(-10, pros::MotorGearset::green);

pros::MotorGroup leftDrive({-18, -19, 20}, pros::MotorGearset::blue);
pros::MotorGroup rightDrive({12, 13, -14}, pros::MotorGearset::blue);
pros::MotorGroup intake({9, 1});

pros::adi::Pneumatics rightDoink = pros::adi::Pneumatics(7, false);
pros::adi::Pneumatics leftDoink = pros::adi::Pneumatics(4, false);
pros::adi::Pneumatics intakeLift = pros::adi::Pneumatics(3, false);
pros::adi::Pneumatics clamp = pros::adi::Pneumatics(8, false);

pros::Rotation horizontalEncoder(17);
pros::Rotation verticalEncoder(-15);
pros::Rotation ladybrownPos(8);

pros::Distance clampDistance(2);
pros::Distance xDistance(6);
pros::Distance yDistance(5);

pros::Optical colorSensor(3);

pros::Imu imu(2);

lemlib::TrackingWheel horizontalTracker(&horizontalEncoder,
										lemlib::Omniwheel::NEW_2,
										-5
);

lemlib::TrackingWheel verticalTracker(&verticalEncoder,
									lemlib::Omniwheel::NEW_2,
									5            
);

lemlib::Drivetrain drivetrain(&leftDrive,
                             &rightDrive,
                             10.63,
                             lemlib::Omniwheel::NEW_325,
                             450,
                             2      
);

lemlib::OdomSensors odom(&verticalTracker,
						nullptr,
						&horizontalTracker,
						nullptr,
						&imu      
);

lemlib::PID ladybrownPID(0.04,
                        0,
                        0.05
);

lemlib::ControllerSettings lateralPID(22,
									0,
									45,
									0,
									1,
									100,
									3,
									500,
									127
);

lemlib::ControllerSettings angularPID(2,
									0,
									10,
									0,
									0,
									0,
									0,
									0,
									127
);

lemlib::ControllerSettings lateralMogoPID(22,
										0,
										45,
										0,
										1,
										100,
										3,
										500,
										127
);

lemlib::ControllerSettings angularMogoPID(2,
										0,
										10,
										0,
										0,
										0,
										0,
										0,
										127
);

lemlib::ExpoDriveCurve throttleCurve(3,
                                     10,
                                     1.019
);

lemlib::ExpoDriveCurve steerCurve(3,
                                  10,
                                  1.019
);

lemlib::Chassis chassis(drivetrain,
                       lateralPID,
                       angularPID,
                       odom,
                       &throttleCurve,
                       &steerCurve
);

lemlib::Chassis mogoChassis(drivetrain,
						   lateralMogoPID,
						   angularMogoPID,
						   odom,
						   &throttleCurve,
						   &steerCurve
);

struct controlSetup {

    bool toggle;
    bool inverseOutput;

    int state = 0;
    int outputType;
    
    controlSetup(int outputType,
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

        if (outputType == MOTOR) {
            if (state && inverseOutput) return -12000;
            else if (state) return 12000;
        }

        if (outputType == DIGITAL) return state;

        return 0;
    }
};

struct recordingSetup {
  	int value;
  	int lastValue;
  	int controlType;
  
  	std::string name;
  	
  	FILE* fileName;
  
  	recordingSetup(FILE* fileName,
  				int controlType)
  				:
  				fileName(fileName),
  				controlType(controlType)
  				{}

  	void update(const std::string& name, int value, int lastValue) {
        if (value != lastValue) {
            if (controlType == MOTOR) fprintf(fileName, "%s.move_voltage(%d);", name.c_str(), value);
            else if (controlType == DIGITAL) fprintf(fileName, "%s.set_value(%d);", name.c_str(), value);
            else fprintf(fileName, "%s=%d;", name.c_str(), value);
        }
    }
};

controlSetup motorControl(MOTOR);
controlSetup motorRevControl(MOTOR, false, true);
controlSetup digitalControl(DIGITAL);
controlSetup digitalToggleControl(DIGITAL, true);

recordingSetup varRecorder(recordings, VAR);
recordingSetup motorRecorder(recordings, MOTOR);
recordingSetup digitalRecorder(recordings, DIGITAL);

void prevAuto() {
    --autoNum;
}

void selectAuto() {
    selecting = false;
}

void nextAuto() {
    ++autoNum;
}

void initialize() {
    pros::lcd::initialize();
    ctrl.clear();
    chassis.calibrate();
    chassis.setPose(0, 0, -1);

    pros::lcd::register_btn0_cb(prevAuto);
    pros::lcd::register_btn1_cb(selectAuto);
    pros::lcd::register_btn2_cb(nextAuto);

    pros::Task dispayLocation([&]() {
        while (true) {
         
            pros::lcd::print(0, "X: %f", chassis.getPose().x);
            pros::lcd::print(1, "Y: %f", chassis.getPose().y);
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);
            
            ctrl.print(1, 0, "X: %f", chassis.getPose().x);
            ctrl.print(2, 0, "Y: %f", chassis.getPose().y);
            ctrl.print(3, 0, "Theta: %f", chassis.getPose().theta);

            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
           
            pros::delay(20);
        }
    });

    pros::Task autonSelector([&]() {
        while(selecting && !TESTMODE) {
            pros::lcd::set_text(4, "Selected Auton:");

            autoNum = (autoNum + 5) % 5;

            switch(autoNum) {
                case 0:
                    pros::lcd::set_text(4, "Skills");
                    break;
                case 1:
                    pros::lcd::set_text(4, "Red Ringside");
                    break;
                case 2:
                    pros::lcd::set_text(4, "Blue Ringside");
                    break;
                case 3:
                    pros::lcd::set_text(4, "Red Goalside");
                    break;
                case 4:
                    pros::lcd::set_text(4, "Blue Goalside");
                    break;
            }
            pros::delay(20);
        }
    });
}

void skills() {}

void redRingSide() {
    // setup
    allyColor = RED;
    chassis.setPose(0, 0, 135);

    // ally stake
    ladybrown.move_voltage(12000);
    pros::delay(1000);

    // clear ally stake
    chassis.moveToPose(-6, 12, 135, 750, {.forwards=false});
    ladybrown.move_voltage(-12000);

    // get ally stake ring

    // goal
    chassis.moveToPose(-3.3, 35.9, 223.2, 1750, {.forwards=false});
    pros::delay(500);

    // auton line rings
    leftDoink.extend();
    spintake = 1;
    mogoChassis.moveToPose(-27.5, 61.53, 334.54, 2000);

    // doink/mid ring
    mogoChassis.moveToPose(-17.6, 39.5, 334.54, 1750, {.forwards=false});
    pros::delay(500);
    leftDoink.retract();
    rightDoink.extend();
    pros::delay(250);
    mogoChassis.moveToPose(-36.2, 31.2, 232.3, 2000);

    // corner ring
    rightDoink.retract();
    mogoChassis.moveToPose(-58.9, 3.64, -231.18, 3000);
    pros::delay(1000);
}

void blueRingSide() {
    // setup
    allyColor = BLUE;
    chassis.setPose(0, 0, 0);
}

void redGoalSide() {
    // setup
    allyColor = RED;
    chassis.setPose(0, 0, 0);
}

void blueGoalSide() {
    // setup
    allyColor = BLUE;
    chassis.setPose(0, 0, 0);
}

void autonomous(void) {
    autoRunning = true;

    pros::Task colorSort([&]() {
        while (autoRunning) {
            colorReading = colorSensor.get_hue();
            secondStageIntake.set_zero_position(0);

            if (colorReading > 140 && colorReading <= 340 && autoNum % 2 == 0) {
                while (secondStageIntake.get_position() < 0.370) intake.move_voltage(12000);
                intake.move_voltage(-12000);

                continue;
            } else if (colorReading < 140 && colorReading >= 340 && autoNum % 2 != 0) {
                while (secondStageIntake.get_position() < 0.370) intake.move_voltage(12000);
                intake.move_voltage(-12000);

                continue;
            }
  
            if (spintake == 1) intake.move_voltage(12000);
            else if (spintake == -1) intake.move_voltage(-12000);
            else intake.move_voltage(0);

            pros::delay(20);
        }
    });

	pros::Task autoClamp([&]() {
		while (autoRunning) {
			if (clampDistance.get_distance() < 15) clamp.extend();
			else clamp.retract();
			pros::delay(20);
		}
	});

	pros::Task ladybrownMove([&]() {
		if (scoreState) ladybrown_target = FAR;
		else if (loadState) ladybrown_target = LOAD;
		else ladybrown_target = IDLE;

		ladybrown.move_voltage(ladybrownPID.update(ladybrown_target - ladybrownPos.get_position()));
		pros::delay(20);
    });

    switch (autoNum) {
        case 0:
            skills();
            break;
        case 1:
            redRingSide();
            break;
        case 2:
            blueRingSide();
            break;
        case 3:
            redGoalSide();
            break;
        case 4:
            blueGoalSide();
            break;
   }
}

void opcontrol() {
    autoRunning = false;
    
    if (!TESTMODE) {
        lemlib::Timer posProtected(31000);
    
        if (recording) {
            recordings = fopen("/usd/recording.txt", "a");
            fprintf(recordings, "");
            fprintf(recordings, "pros::Task ladybrownMove([&](){");
            fprintf(recordings, "while(!autoRunning){if(scoreState)ladybrown_target=FAR;else if(loadState)ladybrown_target=LOAD;else ladybrown_target=IDLE;");
            fprintf(recordings, "ladybrown.move_voltage(ladybrownPID.update(ladybrown_target-ladybrownPos.get_position()));pros::delay(20);}});\n");
        }
    
        while (true) {
            intake.move_voltage(motorControl.use(ctrl.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
                                + motorRevControl.use(ctrl.get_digital(pros::E_CONTROLLER_DIGITAL_A)));
    
            rightDoink.set_value(digitalControl.use(ctrl.get_digital(pros::E_CONTROLLER_DIGITAL_R1)));
            leftDoink.set_value(digitalControl.use(ctrl.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)));
            loadState = digitalToggleControl.use(ctrl.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN));
            scoreState = digitalControl.use(ctrl.get_digital(pros::E_CONTROLLER_DIGITAL_L1));
            clamp.set_value(digitalControl.use(ctrl.get_digital(pros::E_CONTROLLER_DIGITAL_L2)));
    
            leftY = ctrl.get_analog(ANALOG_LEFT_Y);
            rightX = ctrl.get_analog(ANALOG_RIGHT_X);
            chassis.arcade(leftY, rightX, false, 0.75);
    
            if (posProtected.isDone()) ctrl.rumble("**");
    
            pros::delay(20);
    
            if (!recording) continue;
    
            if (ctrl.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
                recording = false;
                fclose(recordings);
            }
    
            varRecorder.update("leftY", leftY, lastLeftY);
            varRecorder.update("rightX", rightX, lastRightX);
            fprintf(recordings, "chassis.arcade(leftY, rightX, false, 0.75);");
            motorRecorder.update("firstStageIntake", firstStageIntake.get_voltage(), lastFirstStageVoltage);
            motorRecorder.update("secondStageIntake", secondStageIntake.get_voltage(), lastSecondStageVoltage);
            varRecorder.update("loadState", loadState, lastLoadState);
            varRecorder.update("scoreState", scoreState, lastScoreState);
            digitalRecorder.update("leftDoink", leftDoink.is_extended(), lastLeftDoink);
            digitalRecorder.update("rightDoink", rightDoink.is_extended(), lastRightDoink);
            digitalRecorder.update("clamp", clamp.is_extended(), lastClamp);
            digitalRecorder.update("intakeLift", intakeLift.is_extended(), lastIntakeLift);
    
            fprintf(recordings, "pros::delay(%f);\n", pros::millis() - lastWrite);
            lastWrite = pros::millis();
        }
    } else {
        // button for turning robot a chosen distance
        // button for driving robot  a chosen distance
        // button for colorsort testing
        // normal drive control
        // buttons to switch display between motor temp, sensor readings, and position log
        // button to save current coords to screen
        // coord log should log coords with label with 1st points at top and last points at bottom
        // Ex: Coords 1: (x=12, y=-24, theta=35)
        // button for PID tuner
    }
}
