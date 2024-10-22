package org.firstinspires.ftc.teamcode.teleop;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;


public class izzyTeleop extends LinearOpMode {
    //Call motors
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private void initialize(){
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
    }

    private void setMotorDirection(){
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    private void telemetry(){
        telemetry.update();
    }

    private void movement(double vertical, double strafe, double turn){
        frontLeft.setPower(vertical + strafe + turn);
        frontRight.setPower(vertical + strafe + turn);
        backLeft.setPower(vertical + strafe + turn);
        backRight.setPower(vertical + strafe + turn);
    }
    @Override
    public void runOpMode() throws InterruptedException {
        //init
        initialize();
        //set motor directions
        setMotorDirection();
        //telemetry
        telemetry();
        waitForStart();
        while (opModeIsActive()){
            //set base speed
            //set slow, fast, and brake
            //movement
        }

    }
}
