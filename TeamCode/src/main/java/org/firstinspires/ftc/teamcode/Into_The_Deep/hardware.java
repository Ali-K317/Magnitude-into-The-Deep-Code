package org.firstinspires.ftc.teamcode.Into_The_Deep;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


public class hardware {
    // Arm components
    public DcMotor mantis; //EXP PRT 0
    public DcMotor lift; //EXP PRT 2
    public DcMotor hopper;
    public DcMotor bar;//EXP PRT 3

    // Wrist and door
    public CRServo bottomGrabber; //EXP PRT 0
    public CRServo topGrabber; //EXP PRT 1
    public Servo door; //EXP PRT 2

    // Wheel components
    public DcMotor frontLeft; //CTRL PRT 0
    public DcMotor frontRight; //CTRL PRT 1
    public DcMotor backLeft; //CTRL PRT 2
    public DcMotor backRight; //CTRL PRT 3
}