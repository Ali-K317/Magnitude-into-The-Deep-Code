package org.firstinspires.ftc.teamcode.teleop.newTeleop;


public interface teleop_interface {
    void initialize();
    void setDirection();

    void telemetry();
    void whileMotorsBusy();

    void movement(double vertical, double strafe, double turn);
    void arm(teleop_enum state, double speed);
    void gripper(int pos);

    void finalMovement();
    void finalArm();
    void finalGrabber();
}
