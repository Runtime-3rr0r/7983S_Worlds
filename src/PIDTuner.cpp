// tune PID

const bool ANGULAR = false; // dont change these
const bool LATERAL = true;

std::string tunePID(const float tune_rate = 1,
                    const float overshoot_threshold = 1,
                    const int timeout = 4000,
                    bool tune_type = false) {
   
    // variables
    const int start_time = pros::millis();

    double kP = 0.01;
    double kD = 0;
    double last_kP = 0.01;
    double last_kD = 0;
    double error = 10;
    double overshoot_distance = 0;
    double lowest_overshoot = 1000;
    double lateral_target = 24 / (2 * M_PI / 360) * 100;

    std::string display_kP = std::to_string(kP);
    std::string display_kD = std::to_string(kD);
    std::string display_lowest_overshoot = std::to_string(lowest_overshoot);
    std::string data = display_kP + display_kD + display_lowest_overshoot;

    int restore_count = 0;
    int consistency_count = 0;
    int current_time = 0;
    int angular_target = 90;

    bool restore_available = false;

    // initial PID define
    lemlib::PID movementPID(kP, 0, kD);

    while (true) {

        while (error != 0 || pros::millis() - start_time >= timeout) {

            if (tune_type) {
                error = lateral_target - vertical_encoder.get_position();
                right_drive.move(-movementPID.update(error));
                left_drive.move(movementPID.update(error));
               
                if (error < 0 && lateral_target > 0) {
                    error += overshoot_distance;
                }
            } else {
                error = lateral_target - imu.get_rotation();
                right_drive.move(movementPID.update(error));
                left_drive.move(movementPID.update(error));

                if (error < 0 && lateral_target > 0) {
                    error += overshoot_distance;
                }
            }
           
            pros::delay(20);
        }

        // update lowest
        if (overshoot_distance < lowest_overshoot) {
            overshoot_distance = lowest_overshoot;
        }

        // update constants
        if (overshoot_distance > lowest_overshoot && restore_available) {
                kP = last_kP;
                kD = last_kD;
        } else {
            if (overshoot_distance > overshoot_threshold) {
                kD += tune_rate * overshoot_distance;
                restore_available = true;
                restore_count += 1;
            } else {
                kP += tune_rate;
                restore_available = false;
                restore_count = 0;
            }
        }
       
        // test for consistency
        if (consistency_count == 5) {
            return data;
        } else if (kP == last_kP && kD == last_kD) {
            consistency_count += 1;
        }
       
        // update
        lemlib::PID movementPID(kP, 0, kD);
        data = display_kP + display_kD + display_lowest_overshoot;
        last_kP = kP;
        last_kD = kD;

        // setup for next run
        overshoot_distance = 0;
        error = 10;
        lateral_target *= -1;
        angular_target *= -1;
        vertical_encoder.reset_position();
        imu.reset(true);
    }
    return 0;
}
