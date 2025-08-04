#pragma once

class PodState {
public:
    double PodSpeed;
    double Angle;

    PodState(double speed = 0.0, double angle = 0.0)
        : PodSpeed(speed), Angle(angle) {}

    //negates the pod speed
    void flip() {
        PodSpeed = -PodSpeed;
    }
};
