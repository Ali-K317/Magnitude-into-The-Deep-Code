package org.firstinspires.ftc.teamcode.autonomous.newAuto.classes;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.ArrayList;
import java.util.List;

public class hardware {
    public DcMotor frontLeft;//ctrl hub prt 0
    public DcMotor frontRight;//ctrl hub prt 1
    public DcMotor backLeft;//crtl hub prt 2
    public DcMotor backRight;//crtl hub prt 3

    public DcMotor mantis;
    public DcMotor hopper;
    public DcMotor lift;

    public Servo grabber;

    public DistanceSensor distanceSensorLeft;//crtl hub I2C prt 1
    public DistanceSensor distanceSensorBack;//crtl hub I2C prt 2
    public ColorSensor colorSensor;//crtl hub I2C 3
    public DistanceSensor distanceSensorRight;//exp hub I2C prt 0

    public ElapsedTime runTime = new ElapsedTime();

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
    public String[] puns = {
            "I'm wired for success!",
            "I’ve got a lot of drive – especially for motors!",
            "I'm always in control, even without a joystick.",
            "You auto believe in yourself!",
            "Stop resisting, it’s time to start conducting!",
            "It’s not just a phase, it’s a current situation.",
            "I’m good at debugging – I’ve got a lot of resistance!",
            "Don’t short-circuit your success!",
            "I torque therefore I am.",
            "Resistance is futile – I’m powered up for this!",
            "Let’s roll, I’m geared up!",
            "Stay positive – your circuit depends on it!",
            "This competition is heating up – hope we don’t overcharge!",
            "I’ve got drive – literally and figuratively!",
            "You’ve got the power… and so do I!",
            "Keep calm and carry on coding.",
            "It’s time to servo up some victories!"
    };
}
