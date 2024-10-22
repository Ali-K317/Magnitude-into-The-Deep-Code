package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp()
public class roseTeleop extends LinearOpMode {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private void initialize(){
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "bacRight");
    }
    private void setDirection(){
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
    }
    private void movement(double vertical, double horizontal, double turn){
        frontLeft.setPower(vertical + horizontal + turn);
        frontRight.setPower(vertical + horizontal + turn);
        backLeft.setPower(vertical + horizontal + turn);
        backRight.setPower(vertical + horizontal + turn);
    }

    public void runOpMode(){
        initialize();
        setDirection();
        waitForStart();
        while(opModeIsActive()){
            double reduction = 0.5;
            double vertical = gamepad1.left_stick_y * reduction;
            double horizontal = gamepad1.left_stick_x * reduction;
            double turn = gamepad1.right_stick_x * reduction;

            movement(vertical, horizontal,turn);
        }
    }
}
