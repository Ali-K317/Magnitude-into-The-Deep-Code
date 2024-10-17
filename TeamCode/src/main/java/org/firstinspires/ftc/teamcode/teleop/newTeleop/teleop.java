package org.firstinspires.ftc.teamcode.teleop.newTeleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

// TeleOp annotation to register this OpMode with the FTC Driver Station
@TeleOp(name = "Into The Deep Teleop", group = "Teleop")
public class teleop extends LinearOpMode implements teleop_interface {
    // Instance of the hardware class to manage robot components
    hardware hardware = new hardware();

    @Override
    public void initialize() {
        // Initialize motors by getting them from the hardware map
        hardware.frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        hardware.frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        hardware.backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        hardware.backRight = hardwareMap.get(DcMotor.class, "backRight");
    }

    @Override
    public void setDirection() {
        // Set the direction of each motor
        hardware.frontLeft.setDirection(DcMotor.Direction.REVERSE);
        hardware.frontRight.setDirection(DcMotor.Direction.FORWARD);
        hardware.backLeft.setDirection(DcMotor.Direction.REVERSE);
        hardware.backRight.setDirection(DcMotor.Direction.FORWARD);
    }

    @Override
    public void movement(Gamepad gamepad, double vertical, double strafe, double turn) {
        // Set power to each motor based on gamepad input
        hardware.frontLeft.setPower(-vertical - strafe - turn);
        hardware.frontRight.setPower(-vertical + strafe + turn);
        hardware.backLeft.setPower(-vertical + strafe - turn);
        hardware.backRight.setPower(-vertical - strafe + turn);
        whileMotorsBusy();
    }

    @Override
    public void telemetry() {
        // Add motor speeds and positions to telemetry
        telemetry.addLine("Robot started");
        telemetry.update();
    }

    @Override
    public boolean whileMotorsBusy() {
        // Monitor motor positions and power while motors are busy
        while (opModeIsActive() && hardware.frontLeft.isBusy() && hardware.frontRight.isBusy() && hardware.backLeft.isBusy() && hardware.backRight.isBusy()) {
            telemetry.addData("Front Left Power", hardware.frontLeft.getPower());
            telemetry.addData("Front Right Power", hardware.frontRight.getPower());
            telemetry.addData("Back Left Power", hardware.backLeft.getPower());
            telemetry.addData("Back Right Power", hardware.backRight.getPower());
            telemetry.update();
            return true;
        }
        return false;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            // Initialize hardware
            initialize();
            telemetry.addLine("Init successful");
            telemetry.update();
        } catch (NullPointerException e) {
            telemetry.addLine("One or more motors returned null");
            telemetry.update();
        }

        // Set motor directions
        setDirection();

        // Wait for the start button to be pressed
        waitForStart();

        // Main loop
        while (opModeIsActive()) {
            // Update telemetry
            telemetry();
            double reduction = 0.4;
            double turnReduction = 0.35;

            // Adjust speeds based on button presses
            if (gamepad1.a) {
                // Slow mode
                reduction = 0.2;
                turnReduction = 0.15;
            } else if (gamepad1.b) {
                // Fast mode
                reduction = 0.6;
                turnReduction = 0.55;
            } else if ((gamepad1.left_stick_button) || (gamepad1.right_stick_button)) {
                // Stop mode
                reduction = 0.0;
                turnReduction = 0.0;
            }

            // Apply movement to motors
            double vertical = reduction * gamepad1.left_stick_y;
            double strafe = -reduction * gamepad1.left_stick_x;
            double turn = -turnReduction * gamepad1.right_stick_x;
            movement(gamepad1, vertical, strafe, turn);
        }
    }
}
