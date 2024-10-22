package org.firstinspires.ftc.teamcode.autonomous.test.interfaces;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.autonomous.test.enums.movement_enum;

public interface movement_interface {
    void moveTo(movement_enum state, HardwareMap hardwareMap);
    void test_forward(double inches, double speed);
}
