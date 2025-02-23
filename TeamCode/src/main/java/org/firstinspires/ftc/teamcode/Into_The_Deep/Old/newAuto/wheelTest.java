package org.firstinspires.ftc.teamcode.Into_The_Deep.Old.newAuto;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Into_The_Deep.hardware;

//@Autonomous(name = "Wheel test", group = "Test")
public class wheelTest extends LinearOpMode {
    hardware hardware = new hardware();

    int tickPer10cm = 100;
    int tickPerCm = (int) tickPer10cm/10;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        setDirection();
        resetEncoders();
        waitForStart();
        while (opModeIsActive()) {
            forward(tickPer10cm, 0.2);
            break;
        }
    }
    private void initialize(){
        hardware.frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        hardware.frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        hardware.backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        hardware.backRight = hardwareMap.get(DcMotor.class, "backRight");
    }
    private void setDirection(){
        hardware.frontLeft.setDirection(DcMotor.Direction.REVERSE);
        hardware.frontRight.setDirection(DcMotor.Direction.FORWARD);
        hardware.backLeft.setDirection(DcMotor.Direction.REVERSE);
        hardware.backRight.setDirection(DcMotor.Direction.FORWARD);
    }
    private void forward(int pos, double speed){
        setPos(pos);
        runToPos();
        while(hardware.frontLeft.getCurrentPosition() < pos){
            setSpeed(speed);
        }
        speed = 0.0;
        setSpeed(speed);
        resetEncoders();
        sleep(250);
    }
    private void setSpeed(double speed){
        hardware.frontLeft.setPower(speed);
        hardware.frontRight.setPower(speed);
        hardware.backLeft.setPower(speed);
        hardware.backRight.setPower(speed);
    }
    private void setPos(int pos){
        hardware.frontLeft.setTargetPosition(pos);
        hardware.frontRight.setTargetPosition(pos);
        hardware.backLeft.setTargetPosition(pos);
        hardware.backRight.setTargetPosition(pos);
    }
    private void resetEncoders(){
        hardware.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    private void runToPos(){
        hardware.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
}
