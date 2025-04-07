#include "main.h"
#include "pros/motor_group.hpp"

int ladybrownTarget = 0;
int currentScreen = 0;
int spintake = 0;
int autoNum = 0;

bool scoreState = false;
bool loadState = false;
bool selecting = true;

pros::Controller ctrl(pros::E_CONTROLLER_MASTER);

pros::adi::Pneumatics rightDoink = pros::adi::Pneumatics(7, false);
pros::adi::Pneumatics intakeLift = pros::adi::Pneumatics(3, false);
pros::adi::Pneumatics leftDoink = pros::adi::Pneumatics(4, false);
pros::adi::Pneumatics clamp = pros::adi::Pneumatics(8, false);

pros::MotorGroup leftDrive({-18, -19, 20}, pros::MotorGearset::blue);
pros::MotorGroup rightDrive({12, 13, -14}, pros::MotorGearset::blue);
pros::MotorGroup intake({9, 1});
pros::MotorGroup ladybrown({-10, -21}, pros::MotorGearset::green);

pros::Motor firstStageIntake(9, pros::MotorGearset::green);
pros::Motor secondStageIntake(1, pros::MotorGearset::blue);
pros::Motor ladybrownLeft(-10, pros::MotorGearset::green);
pros::Motor ladybrownRight(-21, pros::MotorGearset::green);

pros::Rotation horizontalEncoder(17);
pros::Rotation verticalEncoder(-15);
pros::Rotation ladybrownPos(8);

pros::Distance clampDistance(2);
pros::Distance xDistance(6);
pros::Distance yDistance(5);

pros::Optical colorSensor(3);

pros::Imu imu(2);

std::list<std::string> coordLog;
FILE* recordings = nullptr;
std::string coordLogEntry;

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

ControlSetup motorControl(MOTOR);
ControlSetup motorRevControl(MOTOR, false, true);
ControlSetup digitalControl(DIGITAL);
ControlSetup digitalToggleControl(DIGITAL, true);

RecordingSetup varRecorder(recordings, VAR);
RecordingSetup motorRecorder(recordings, MOTOR);
RecordingSetup digitalRecorder(recordings, DIGITAL);

std::string tunePID(bool tuneType = LATERAL) {

    const int start_time = pros::millis();

    float kP = 0.01;
    float kD = 0;
    float lastKP = 0.01;
    float lastKD = 0;
    float error = 10;
    float overshootDistance = 0;
    float lowestOvershoot = 1000;
    float lateralTarget = 24 / (2 * M_PI / 360) * 100;

    std::string displayKP = std::to_string(kP);
    std::string displayKD = std::to_string(kD);
    std::string displayLowestOvershoot = std::to_string(lowestOvershoot);
    std::string data = displayKP + ", " + displayKD + ", " + displayLowestOvershoot;

    int restoreCount = 0;
    int consistencyCount = 0;
    int angularTarget = 90;

    bool restoreAvailable = false;
    bool tunerEnabled = true;

    lemlib::PID movementPID(kP, 0, kD);

    while (tunerEnabled) {
        while (error != 0 || pros::millis() - start_time < TIMEOUT) {

            if (ctrl.get_digital(pros::E_CONTROLLER_DIGITAL_B)) tunerEnabled = false;

            if (tuneType == LATERAL) {
                error = lateralTarget - verticalEncoder.get_position();
                rightDrive.move(-movementPID.update(error));
                leftDrive.move(movementPID.update(error));

                if (error < 0 && lateralTarget > 0) error += overshootDistance;
            } else {
                error = lateralTarget - imu.get_rotation();
                rightDrive.move(movementPID.update(error));
                leftDrive.move(movementPID.update(error));

                if (error < 0 && lateralTarget > 0) error += overshootDistance;
            }
            pros::delay(20);
        }

        if (overshootDistance < lowestOvershoot) overshootDistance = lowestOvershoot;

        if (overshootDistance > lowestOvershoot && restoreAvailable) {
            kP = lastKP;
            kD = lastKD;
        } else {
            if (overshootDistance > THRESHOLD) {
                kD += TUNERATE * overshootDistance;
                restoreAvailable = true;
                restoreCount += 1;
            } else {
                kP += TUNERATE;
                restoreAvailable = false;
                restoreCount = 0;
            }
        }

        if (consistencyCount == 5) return data;
        else if (kP == lastKP && kD == lastKD) consistencyCount += 1;

        lemlib::PID movementPID(kP, 0, kD);
        displayKP = std::to_string(kP);
        displayKD = std::to_string(kD);
        displayLowestOvershoot = std::to_string(lowestOvershoot);
        data = displayKP + ", " + displayKD + ", " + displayLowestOvershoot;
        lastKP = kP;
        lastKD = kD;

        overshootDistance = 0;
        error = 10;
        lateralTarget *= -1;
        angularTarget *= -1;
        verticalEncoder.reset_position();
        imu.reset(true);
    }
    return data;
} 

bool autoRunning = false;
bool recording = false;

void prevAuto() {
    --autoNum;
}

void selectAuto() {
    selecting = false;
}

void nextAuto() {
    ++autoNum;
}

void nextScreen() {
    currentScreen = (currentScreen + 1) % 3;
}

void prevScreen() {
    currentScreen = (currentScreen + 2) % 3;
}

void initialize() {
    pros::lcd::initialize();
    ctrl.clear();
    chassis.calibrate();
    chassis.setPose(0, 0, 0);

    pros::lcd::register_btn0_cb(prevAuto);
    pros::lcd::register_btn1_cb(selectAuto);
    pros::lcd::register_btn2_cb(nextAuto);

    pros::Task displayLocation([&]() {
        while (true) {
        
            if (!autoRunning) {
                pros::lcd::print(0, "X: %f", chassis.getPose().x);
                pros::lcd::print(1, "Y: %f", chassis.getPose().y);
                pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);    
            }
            
            if (!recording) {
                ctrl.print(1, 0, "X: %f", chassis.getPose().x);
                ctrl.print(2, 0, "Y: %f", chassis.getPose().y);
                ctrl.print(3, 0, "Theta: %f", chassis.getPose().theta);
            }
            
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

void skills() {
    // setup
    chassis.setPose(0, 0, 135);
}

void redRingSide() {
    // setup
    chassis.setPose(0, 0, 135);

    // ally stake
    ladybrownTarget = FAR;
    pros::delay(1000);

    // clear ally stake
    chassis.moveToPose(-6, 12, 135, 750, {.forwards=false});
    ladybrownTarget = IDLE;
    pros::delay(500);

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
    chassis.setPose(0, 0, 0);
}

void redGoalSide() {
    // setup
    chassis.setPose(0, 0, 0);
}

void blueGoalSide() {
    // setup
    chassis.setPose(0, 0, 0);
}

void autonomous(void) {
    int movementStartTime = 0;
    int moveNum = 0;
    
    pros::lcd::clear();
    
    autoRunning = true;

    pros::Task movementTimer([&](){
        while (autoRunning) {
            if (chassis.isInMotion() || mogoChassis.isInMotion() && !movementStartTime) {
                movementStartTime = pros::millis();
            }

            if (!chassis.isInMotion() && !mogoChassis.isInMotion() && movementStartTime) {
                pros::lcd::print(moveNum, "Move Time: %f", pros::millis() - movementStartTime);
                moveNum += 1;
                movementStartTime = 0;
            }
            pros::delay(20);
        }
    });

    pros::Task colorSort([&]() {
        while (autoRunning) {
            secondStageIntake.set_zero_position(0);

            if (colorSensor.get_hue() > 140 && colorSensor.get_hue() <= 340 && autoNum % 2 == 0) {
                while (secondStageIntake.get_position() < 0.370) intake.move_voltage(12000);
                intake.move_voltage(-12000);
                continue;
            } else if (colorSensor.get_hue() < 140 && colorSensor.get_hue() >= 340 && autoNum % 2 != 0 || autoNum == 0) {
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
		if (scoreState) ladybrownTarget = FAR;
		else if (loadState) ladybrownTarget = LOAD;
		else ladybrownTarget = IDLE;

		ladybrown.move_voltage(ladybrownPID.update(ladybrownTarget - ladybrownPos.get_position()));
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
    pros::lcd::register_btn0_cb(prevScreen);
    pros::lcd::register_btn1_cb(nextScreen);
    
    int lastSecondStageVoltage = 0;
    int lastFirstStageVoltage = 0;
    int lastRightX = 0;
    int lastLeftY = 0;
    int lastWrite = 0;
    int coordLogItem = 0;
    int holdCount = 0;
    int tuneType = 0;
    int rightX = 0;
    int leftY = 0;

    bool lastScoreState = false;
    bool lastRightDoink = false;
    bool lastIntakeLift = false;
    bool lastLoadState = false;
    bool lastLeftDoink = false;
    bool lastClamp = false;
    
    lemlib::Timer posProtected(31000);

    if (recording) {
        recordings = fopen("/usd/recording.txt", "a");
        fprintf(recordings, "");
        fprintf(recordings, "pros::Task ladybrownMove([&](){");
        fprintf(recordings, "while(!autoRunning){if(scoreState)ladybrownTarget=FAR;else if(loadState)ladybrownTarget=LOAD;else ladybrownTarget=IDLE;");
        fprintf(recordings, "ladybrown.move_voltage(ladybrownPID.update(ladybrownTarget-ladybrownPos.get_position()));pros::delay(20);}});\n");
    }

    while (true) {
        intake.move_voltage(motorControl.use(ctrl.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
                            + motorRevControl.use(ctrl.get_digital(pros::E_CONTROLLER_DIGITAL_A)));

        rightDoink.set_value(digitalControl.use(ctrl.get_digital(pros::E_CONTROLLER_DIGITAL_R1)));
        leftDoink.set_value(digitalControl.use(ctrl.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)));
        loadState = digitalToggleControl.use(ctrl.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN));
        scoreState = digitalControl.use(ctrl.get_digital(pros::E_CONTROLLER_DIGITAL_L1));
        clamp.set_value(digitalControl.use(ctrl.get_digital(pros::E_CONTROLLER_DIGITAL_L2)));

        leftY = ctrl.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        rightX = ctrl.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        chassis.arcade(leftY, rightX, false, 0.75);

        if (posProtected.isDone()) ctrl.rumble("*-*");

        if (ctrl.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
            recording = !recording;
            ctrl.rumble("**");
            
            if (recording && recordings == nullptr) recordings = fopen("/usd/recording.txt", "a");
            else if (!recording && recordings != nullptr) fclose(recordings);
        }

        if (recording) {
            lastLeftY = varRecorder.update("leftY", leftY, lastLeftY);
            lastRightX = varRecorder.update("rightX", rightX, lastRightX);
            fprintf(recordings, "chassis.arcade(leftY, rightX, false, 0.75);");
            lastFirstStageVoltage = motorRecorder.update("firstStageIntake", firstStageIntake.get_voltage(), lastFirstStageVoltage);
            lastSecondStageVoltage = motorRecorder.update("secondStageIntake", secondStageIntake.get_voltage(), lastSecondStageVoltage);
            lastLoadState = varRecorder.update("loadState", loadState, lastLoadState);
            lastScoreState = varRecorder.update("scoreState", scoreState, lastScoreState);
            lastLeftDoink = digitalRecorder.update("leftDoink", leftDoink.is_extended(), lastLeftDoink);
            lastRightDoink = digitalRecorder.update("rightDoink", rightDoink.is_extended(), lastRightDoink);
            lastClamp = digitalRecorder.update("clamp", clamp.is_extended(), lastClamp);
            lastIntakeLift = digitalRecorder.update("intakeLift", intakeLift.is_extended(), lastIntakeLift);
    
            fprintf(recordings, "pros::delay(%d);\n", pros::millis() - lastWrite);
            lastWrite = pros::millis();
        }
        
        pros::delay(20);

        if (!TESTMODE) continue;

        if (ctrl.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            ctrl.clear();
            if (tuneType == LATERAL) ctrl.print(1, 0, "PID Tuner: Lateral");
            else ctrl.print(1, 0, "PID Tuner: Angular");
            
            holdCount = 0;
            while (ctrl.get_digital(pros::E_CONTROLLER_DIGITAL_B) && holdCount < 500) {
                holdCount += 20;
                pros::delay(20);
            }
            
            if (holdCount >= 500) ctrl.print(2, 0, "%s", tunePID(tuneType));
            else tuneType = !tuneType;
        }

        if (ctrl.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) currentScreen = (currentScreen + 1) % 3;

        if (currentScreen == 0) {
            pros::lcd::clear();
            pros::lcd::print(0, "Motor Temps:");
            pros::lcd::print(2, "First Stage: %f", firstStageIntake.get_temperature());
            pros::lcd::print(3, "Second Stage: %f", secondStageIntake.get_temperature());
            pros::lcd::print(4, "Ladybrown: %f", ladybrown.get_temperature());
            pros::lcd::print(5, "Left Drive: %f", leftDrive.get_temperature());
            pros::lcd::print(6, "Right Drive: %f", rightDrive.get_temperature());
        } else if (currentScreen == 1) {
            pros::lcd::clear();
            pros::lcd::print(0, "Sensor Readings:");
            pros::lcd::print(2, "Color Sensor: %f", colorSensor.get_hue());
            pros::lcd::print(3, "Clamp Distance: %f", clampDistance.get_distance());
            pros::lcd::print(4, "X Distance: %f", xDistance.get_distance());
            pros::lcd::print(5, "Y Distance: %f", yDistance.get_distance());
            pros::lcd::print(6, "Ladybrown Position: %f", ladybrownPos.get_position());
            pros::lcd::print(7, "Horizontal Encoder: %f", horizontalEncoder.get_position());
            pros::lcd::print(8, "Vertical Encoder: %f", verticalEncoder.get_position());
        } else {
            pros::lcd::clear();
            pros::lcd::print(0, "Coord Log:");

            if (ctrl.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) coordLogItem = (coordLogItem + 1) % coordLog.size();

            int line = 2;
            auto it = coordLog.begin();
            std::advance(it, coordLogItem);

            for (int i = 0; i < 10 && it != coordLog.end(); ++i, ++it, ++line) pros::lcd::print(line, "%s", it->c_str());
        }

        if (ctrl.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
            coordLogItem = coordLog.size();
            coordLogEntry = std::to_string(coordLogItem)
                            + "x: " + std::to_string(chassis.getPose().x) + ", "
                            + "y: " + std::to_string(chassis.getPose().y) + ", "
                            + "theta: " + std::to_string(chassis.getPose().theta);
            coordLog.push_back(coordLogEntry);
        }
    }
}
