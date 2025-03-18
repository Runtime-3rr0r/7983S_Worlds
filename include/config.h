pros::Controller ctrl(pros::E_CONTROLLER_MASTER);

pros::adi::Pneumatics rightDoink = pros::adi::Pneumatics(7, false);
pros::adi::Pneumatics intakeLift = pros::adi::Pneumatics(3, false);
pros::adi::Pneumatics leftDoink = pros::adi::Pneumatics(4, false);
pros::adi::Pneumatics clamp = pros::adi::Pneumatics(8, false);

pros::MotorGroup leftDrive({-18, -19, 20}, pros::MotorGearset::blue);
pros::MotorGroup rightDrive({12, 13, -14}, pros::MotorGearset::blue);
pros::MotorGroup intake({9, 1});

pros::Motor firstStageIntake(9, pros::MotorGearset::green);
pros::Motor secondStageIntake(1, pros::MotorGearset::blue);
pros::Motor ladybrown(-10, pros::MotorGearset::green);

pros::Rotation horizontalEncoder(17);
pros::Rotation verticalEncoder(-15);
pros::Rotation ladybrownPos(8);

pros::Distance clampDistance(2);
pros::Distance xDistance(6);
pros::Distance yDistance(5);

pros::Optical colorSensor(3);

pros::Imu imu(2);
