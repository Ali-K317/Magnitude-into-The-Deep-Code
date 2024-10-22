package org.firstinspires.ftc.teamcode.autonomous.newAuto;

import com.qualcomm.robotcore.hardware.DcMotor;

public interface newAuto_interface {
    void initialize();
    void telemetry();
    void setDirection();

    void runToPosition(DcMotor motor);
    void motorToPosition();

    void resetEncoder(DcMotor motor);
    void resetMotorEncoders();

    void setBrakes(DcMotor motor);
    void setMotorBrakes();

    void setPosition(int frontLeftPos, int frontRightPos, int backLeftPos, int backRightPos);
    void setSpeed(double frontLeftSpeed, double frontRightSpeed, double backRightSpeed, double backLeftSpeed);

    boolean whileMotorsBusy();

    void base(int targetPosFL, int targetPosFR, int targetPosBL, int targetPosBR, double speedFL, double speedFR, double speedBL, double speedBR);
    void movement(newAuto_enum state, double inches, double speed);
    void turn(newAuto_enum state, double degrees, double speed);
    void arm(newAuto_enum motor, double inch, double speed);

    boolean detectYellow(int blue, int red, int green);

    void moveTo(newAuto_enum state);
}
