package org.firstinspires.ftc.teamcode.autonomous.newAuto.classes;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class hardware {
    public DcMotor frontLeft;//ctrl hub prt 0
    public DcMotor frontRight;//ctrl hub prt 1
    public DcMotor backLeft;//crtl hub prt 2
    public DcMotor backRight;//crtl hub prt 3

    public DcMotor mantis;
    public DcMotor lift;

    public DistanceSensor distanceSensorLeft;//crtl hub I2C prt 1
    public DistanceSensor distanceSensorBack;//crtl hub I2C prt 2
    public ColorSensor colorSensor;//crtl hub I2C 3
    public DistanceSensor distanceSensorRight;//exp hub I2C prt 0

    public ElapsedTime runTime = new ElapsedTime();
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


    public void init(HardwareMap hardwareMap) {
        // Initialize individual motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

    }
    public DcMotor[] motors = new DcMotor[]{frontLeft,frontRight,backLeft,backRight};
}
