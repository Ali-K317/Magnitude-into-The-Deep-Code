package org.firstinspires.ftc.teamcode.autonomous.newAuto;

import com.qualcomm.robotcore.hardware.DcMotor;

public interface newAuto_interface {
    void initialize();
    void telemetry();
    void setMode();
    void resetEncoders();
    void resetMotorEncoder(DcMotor motor);
    void setDirection();
    void setBrakes();
    void setMotorBrakes(DcMotor motor);
    void setPosition(int frontLeftPos, int frontRightPos, int backLeftPos, int backRightPos);
    void setSpeed(double frontLeftSpeed, double frontRightSpeed, double backRightSpeed, double backLeftSpeed);
    boolean whileMotorsBusy();
    void base(int targetPosFL, int targetPosFR, int targetPosBL, int targetPosBR, double speedFL, double speedFR, double speedBL, double speedBR);
    void arm(newAuto_enum motor, double inch, double speed);
    void movement(newAuto_enum state, double inches, double speed);
    boolean detectYellow(int blue, int red, int green);


    void moveTo(newAuto_enum state);
    void turn(newAuto_enum state, double degrees, double speed);



}
