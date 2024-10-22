package org.firstinspires.ftc.teamcode.autonomous.test.classes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.autonomous.test.enums.movement_enum;
import org.firstinspires.ftc.teamcode.autonomous.test.interfaces.movement_interface;


public class movement extends LinearOpMode implements movement_interface {
    org.firstinspires.ftc.teamcode.autonomous.test.classes.hardware hardware;
    org.firstinspires.ftc.teamcode.autonomous.test.classes.calculations calculations;

    public movement(hardware hardware, HardwareMap hardwareMap){
        this.hardware = hardware;
    }



    @Override
    public void moveTo(movement_enum state, HardwareMap hardwareMap) {
        calculations = new calculations();
        hardware.initialize(hardwareMap);
        hardware.setMode();
        switch (state){
            case START_POSITION:
                test_forward(1.0,calculations.DRIVESPEED);
                break;
            case GO_TO_BASKET:
                test_forward(2.0,calculations.DRIVESPEED);
                break;
            case END_POSITION:
                test_forward(3.0,calculations.DRIVESPEED);
                break;
        }
    }


    @Override
    public void test_forward(double inches, double speed) {
        telemetry.addLine("Movement is working");
        telemetry.update();

        int targetEncoderCount = (int)(1.0);
        hardware.setPosition(targetEncoderCount);
        hardware.runToPosition();
        hardware.frontLeft.setPower(speed);
        hardware.frontRight.setPower(speed);
        hardware.backLeft.setPower(speed);
        hardware.backRight.setPower(speed);

        while (opModeIsActive() && (hardware.frontLeft.isBusy() && hardware.frontRight.isBusy() && hardware.backLeft.isBusy() && hardware.backRight.isBusy())) {

        }
        hardware.stopAndResetEncoders();
        hardware.frontLeft.setPower(-0.4);
        hardware.frontRight.setPower(-0.4);
        hardware.backLeft.setPower(0.3);
        hardware.backRight.setPower(0.3);
        while(opModeIsActive() && hardware.runtime.seconds() < 1.0){

        }

    }


    @Override
    public void runOpMode() throws InterruptedException {

    }
}
