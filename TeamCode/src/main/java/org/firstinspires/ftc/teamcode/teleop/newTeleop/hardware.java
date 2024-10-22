package org.firstinspires.ftc.teamcode.teleop.newTeleop;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

public class hardware {
    public DcMotor frontLeft; // ctrl hub prt 0
    public DcMotor frontRight; //ctrl hub prt 1
    public DcMotor backLeft; //ctrl hub prt 2
    public DcMotor backRight; //ctrl hun prt 3

    public DcMotor hopper;
    public DcMotor mantis;
    public DcMotor lift;

    public Servo grabber;

    public ColorSensor colorSensor;

    public DistanceSensor distanceSensorLeft;
    public DistanceSensor distanceSensorRight;
    public DistanceSensor distanceSensorBack;


    public void checkMotorInit(){
        // Check if motors are initialized
        if (frontLeft == null) {
            throw new NullPointerException("Front Left Motor is not initialized.");
        }
        if (frontRight == null) {
            throw new NullPointerException("Front Right Motor is not initialized.");
        }
        if (backLeft == null) {
            throw new NullPointerException("Back Left Motor is not initialized.");
        }
        if (backRight == null) {
            throw new NullPointerException("Back Right Motor is not initialized.");
        }

        if (mantis == null){
            throw new NullPointerException("Mantis arm is not initialized");
        }
        if (lift == null){
            throw new NullPointerException("Lift motor not initialized");
        }
        if (hopper == null){
            throw new NullPointerException("Hopper motor not initialized");
        }
        if (grabber == null){
            throw new NullPointerException("Grabber servo not initialized");
        }

        if(colorSensor == null){
            throw new NullPointerException("Color Sensor is not initialized");
        }
        if(distanceSensorRight == null){
            throw new NullPointerException("Right distance sensor not initialized");
        }
        if(distanceSensorLeft == null){
            throw new NullPointerException("Left distance sensor not initialized");
        }
        if(distanceSensorBack == null){
            throw new NullPointerException("Back distance sensor not initialized");
        }
    }

}
