package org.firstinspires.ftc.teamcode.Into_The_Deep.Old.Final.TeleOp;


import org.firstinspires.ftc.teamcode.Into_The_Deep.mainEnum;

public interface teleop_interface {
    void initializeArms();
    void initializeWheels();

    void setDirectionArms();
    void setDirectionWheels();

    void setBrakesArms();
    void setBrakesWheels();

    void telemetry();

    void movement(double vertical, double strafe, double turn);
    void finalMovement();

    void arm(mainEnum state, double speed);
    void finalArm();

    void claw(mainEnum state, double speed, int pos);
    void finalGrabber();
}
