#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "lemlib/pid.hpp"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"

// settings/vars
const bool record = false;
const int RED = 1;
const int BLUE = -1;
const int MOTOR = 1;
const int DIGITAL = 1;
const int IDLE = 2000;
const int LOAD = 4800;
const int SCORE = 16000;
const int FAR = 22800;

int autoNum = 0;
int allyColor = 0;
int ladybrown_target = 0;

bool selecting = true;
bool loadState = false;
bool scoreState = false;

float ladybrownErr = 0;
float lastWrite = 0;

pros::Controller ctrl(pros::E_CONTROLLER_MASTER);

pros::Motor ladybrown(-10, pros::MotorGearset::green);

pros::Motor firstStageIntake(9, pros::MotorGearset::green);
pros::Motor secondStageIntake(1, pros::MotorGearset::blue);

pros::Motor Lf(-18, pros::v5::MotorGears::blue);
pros::Motor Lm(-19, pros::v5::MotorGears::blue);
pros::Motor Lb(20, pros::v5::MotorGears::blue);
pros::Motor Rf(12, pros::v5::MotorGears::blue);
pros::Motor Rm(13, pros::v5::MotorGears::blue);
pros::Motor Rb(-14, pros::v5::MotorGears::blue);

pros::MotorGroup leftDrive({-18, -19, 20}, pros::MotorGearset::blue);
pros::MotorGroup rightDrive({12, 13, -14}, pros::MotorGearset::blue);
pros::MotorGroup intake({9, 1});

pros::adi::Pneumatics rightDoink = pros::adi::Pneumatics(7, false);
pros::adi::Pneumatics leftDoink = pros::adi::Pneumatics(4, false);
pros::adi::Pneumatics clamp = pros::adi::Pneumatics(8, false);
pros::adi::Pneumatics intakeLift = pros::adi::Pneumatics(3, false);

pros::Imu imu(2);
pros::Rotation horizontalEncoder(17);
pros::Rotation verticalEncoder(-15);

pros::Optical colorSensor(3);

pros::Rotation ladybrownPos(8);

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

struct driverControl {

    bool toggle;
    bool inverseOutput;

    int state = 0;
    int outputType;
    
    driverControl(int outputType,
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
            if (state && inverseOutput) return -127;
            else if (state) return 127;
        }
        
        if (outputType == DIGITAL) return state;
        
        return 0;
    }
};

driverControl intakeFwdControl(MOTOR);
driverControl intakeRevControl(MOTOR, false, true);
driverControl rightDoinkControl(DIGITAL);
driverControl leftDoinkControl(DIGITAL);
driverControl clampControl(DIGITAL);
driverControl loadStateControl(DIGITAL, true);
driverControl scoreStateController(DIGITAL);

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
           
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
           
            pros::delay(20);
        }
    });

    pros::Task autonSelector([&]() {
        while(selecting) {
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

int spintake = 0;

void skills() {}

void redRingSide() {
    // setup
    allyColor = RED;
    chassis.setPose(0, 0, 135);

    // ally stake
    ladybrown.move(127);
    pros::delay(1000);

    // clear ally stake
    chassis.moveToPose(-6, 12, 135, 750, {.forwards=false});
    ladybrown.move(-127);

    // get ally stake ring

    // goal
    chassis.moveToPose(-3.3, 35.9, 223.2, 1750, {.forwards=false});
    pros::delay(2250);
    clamp.extend();
    pros::delay(500);

    // auton line rings
    leftDoink.extend();
    spintake = 1;
    chassis.moveToPose(-27.5, 61.53, 334.54, 2000);

    // doink/mid ring
    chassis.moveToPose(-17.6, 39.5, 334.54, 1750, {.forwards=false});
    pros::delay(500);
    leftDoink.retract();
    rightDoink.extend();
    pros::delay(250);
    chassis.moveToPose(-36.2, 31.2, 232.3, 2000);

    // corner ring
    rightDoink.retract();
    chassis.moveToPose(-58.9, 3.64, -231.18, 3000);
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

bool autoRunning = false;
void autonomous(void) {
    autoRunning = true;

    pros::Task colorSort([&]() {
   
        while (autoRunning) {
            float value = colorSensor.get_hue();
  
            if (value > 140 && value <= 340 && allyColor == 1) {
                secondStageIntake.set_zero_position(0);
                 
                while (secondStageIntake.get_position() < 0.370) intake.move(127);
                intake.move(-127);

                continue;
            }
  
            if (spintake == 1) intake.move(127);
            else if (spintake == -1) intake.move(-127);
            else intake.move(0);

            pros::delay(20);
        }
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

    FILE* recordings = fopen("/usd/recording.txt", "w");
        fprintf(recordings, "");
        fclose(recordings);

    if (record) {
        recordings = fopen("/usd/recording.txt", "a");
        fprintf(recordings, "pros::Task ladybrownMove([&](){");
        fprintf(recordings, "while(!autoRunning){if(scoreState)ladybrown_target=FAR;else if(loadState)ladybrown_target=LOAD;else ladybrown_target=IDLE;");
        fprintf(recordings, "ladybrown.move(ladybrownPID.update(ladybrown_target-ladybrownPos.get_position()));pros::delay(20);}});\n");
        fclose(recordings);
    }

    pros::Task ladybrownMove([&]() {
        while (!autoRunning) {
            if (scoreState) ladybrown_target = FAR;
            else if (loadState) ladybrown_target = LOAD;
            else ladybrown_target = IDLE;

            ladybrown.move(ladybrownPID.update(ladybrown_target - ladybrownPos.get_position()));
            pros::delay(20);
        }
    });

    while (true) {

        intake.move(intakeFwdControl.use(ctrl.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
                    + intakeRevControl.use(ctrl.get_digital(pros::E_CONTROLLER_DIGITAL_A)));

        rightDoink.set_value(rightDoinkControl.use(ctrl.get_digital(pros::E_CONTROLLER_DIGITAL_R1)));

        rightDoink.set_value(rightDoinkControl.use(ctrl.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)));

        clamp.set_value(clampControl.use(ctrl.get_digital(pros::E_CONTROLLER_DIGITAL_L2)));
        
        loadState = loadStateControl.use(ctrl.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN));
        scoreState = scoreStateController.use(ctrl.get_digital(pros::E_CONTROLLER_DIGITAL_L1));

        int left_y = ctrl.get_analog(ANALOG_LEFT_Y);
        int right_x = ctrl.get_analog(ANALOG_RIGHT_X);
        chassis.arcade(left_y, right_x, false, 0.75);

        pros::delay(20);

        if (!record) continue;

        recordings = fopen("/usd/recording.txt", "a");

        fprintf(recordings, "Lf.move_voltage(%d);", Lf.get_voltage());
        fprintf(recordings, "Lm.move_voltage(%d);", Lm.get_voltage());
        fprintf(recordings, "Lb.move_voltage(%d);", Lb.get_voltage());
        fprintf(recordings, "Rf.move_voltage(%d);", Rf.get_voltage());
        fprintf(recordings, "Rm.move_voltage(%d);", Rm.get_voltage());
        fprintf(recordings, "Rb.move_voltage(%d);", Rb.get_voltage());
        fprintf(recordings, "firstStageIntake.move_voltage(%d);", firstStageIntake.get_voltage());
        fprintf(recordings, "secondStageIntake.move_voltage(%d);", secondStageIntake.get_voltage());
        fprintf(recordings, "loadState = %d;", loadState);
        fprintf(recordings, "scoreState = %d;", scoreState);
        fprintf(recordings, "leftDoink.set_value(%d);", leftDoink.is_extended());
        fprintf(recordings, "rightDoink.set_value(%d);", rightDoink.is_extended());
        fprintf(recordings, "clamp.set_value(%d);", clamp.is_extended());
        fprintf(recordings, "intakeLift.set_value(%d);", intakeLift.is_extended());

        fprintf(recordings, "pros::delay(%f);\n", pros::millis() - lastWrite);
        lastWrite = pros::millis();
        fclose(recordings);
    }
}