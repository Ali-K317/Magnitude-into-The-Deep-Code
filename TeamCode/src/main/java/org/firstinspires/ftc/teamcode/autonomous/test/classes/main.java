package org.firstinspires.ftc.teamcode.autonomous.test.classes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.autonomous.test.enums.movement_enum;


public class main extends LinearOpMode {
    org.firstinspires.ftc.teamcode.autonomous.test.classes.movement movement;
    org.firstinspires.ftc.teamcode.autonomous.test.classes.calculations calculations;
    org.firstinspires.ftc.teamcode.autonomous.test.classes.hardware hardware;


    @Override
    public void runOpMode() throws InterruptedException {
        try {
            hardware.frontRight = hardwareMap.get(DcMotor.class, "frontRight");
            hardware.frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
            hardware.backLeft = hardwareMap.get(DcMotor.class, "backLeft");
            hardware.backRight = hardwareMap.get(DcMotor.class, "backRight");
        } catch (NullPointerException e) {
            telemetry.addLine("Initial initialization returned null");
            telemetry.update();
        }
        waitForStart();
        while (hardware.runtime.seconds() < 30.0 && opModeIsActive()) {
            telemetry.addLine("Main is working");
            telemetry.update();

            try {
                movement.moveTo(movement_enum.START_POSITION, hardwareMap);
            } catch (NullPointerException e) {
                telemetry.addLine("Movement initialization returned null");
                telemetry.update();

            }
        }
    }
}