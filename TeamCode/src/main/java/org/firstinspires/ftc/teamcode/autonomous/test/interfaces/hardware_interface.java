package org.firstinspires.ftc.teamcode.autonomous.test.interfaces;

import com.qualcomm.robotcore.hardware.HardwareMap;

public interface hardware_interface {
    void setMode();
    void initialize(HardwareMap hardwareMap);
    void setDirection();
    void stopAndResetEncoders();
    void setPosition(int targetEncoderCount);
    void runToPosition();
    void setBrake();
}
