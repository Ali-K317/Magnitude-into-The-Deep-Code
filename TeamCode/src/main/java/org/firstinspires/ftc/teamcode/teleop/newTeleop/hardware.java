package org.firstinspires.ftc.teamcode.teleop.newTeleop;

import com.qualcomm.robotcore.hardware.DcMotor;

public class hardware {
    public DcMotor frontLeft; // ctrl hub prt 0
    public DcMotor frontRight; //ctrl hub prt 1
    public DcMotor backLeft; //ctrl hub prt 2
    public DcMotor backRight; //ctrl hun prt 3
    public DcMotor[] motors = new DcMotor[]{frontLeft,frontRight,backLeft,backRight};

}
