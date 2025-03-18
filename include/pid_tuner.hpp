#include "main.h"

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
