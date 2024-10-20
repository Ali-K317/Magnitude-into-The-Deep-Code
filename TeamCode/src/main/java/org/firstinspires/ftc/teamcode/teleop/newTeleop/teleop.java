package org.firstinspires.ftc.teamcode.teleop.newTeleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

// TeleOp annotation to register this OpMode with the FTC Driver Station
@TeleOp(name = "Into The Deep Teleop", group = "Teleop")
public class teleop extends LinearOpMode implements teleop_interface {
    // Instance of the hardware class to manage robot components
    hardware hardware = new hardware();

    @Override
    public void initialize() {
        // Initialize motors and servos by getting them from the hardware map
        try {
            hardware.frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
            telemetry.addLine("Front Left init successful");
            telemetry.update();
        } catch (NullPointerException e) {
            telemetry.addLine("Front left motor failed to init");
            telemetry.update();
        }

        try {
            hardware.frontRight = hardwareMap.get(DcMotor.class, "frontRight");
            telemetry.addLine("Front Right init successful");
            telemetry.update();
        } catch (NullPointerException e) {
            telemetry.addLine("Front right motor failed to init");
            telemetry.update();
        }

        try {
            hardware.backLeft = hardwareMap.get(DcMotor.class, "backLeft");
            telemetry.addLine("Back Left init successful");
            telemetry.update();
        } catch (NullPointerException e) {
            telemetry.addLine("Back left motor failed to init");
            telemetry.update();
        }

        try {
            hardware.backRight = hardwareMap.get(DcMotor.class, "backRight");
            telemetry.addLine("Back Right init successful");
            telemetry.update();
        } catch (NullPointerException e) {
            telemetry.addLine("Back right motor failed to init");
            telemetry.update();
        }

        try {
            hardware.lift = hardwareMap.get(DcMotor.class, "lift");
            telemetry.addLine("Lift init successful");
            telemetry.update();
        } catch (NullPointerException e) {
            telemetry.addLine("Lift motor failed to init");
            telemetry.update();
        }

        try {
            hardware.hopper = hardwareMap.get(DcMotor.class, "hopper");
            telemetry.addLine("Hopper init successful");
            telemetry.update();
        } catch (NullPointerException e) {
            telemetry.addLine("Hopper motor failed to init");
            telemetry.update();
        }

        try {
            hardware.mantis = hardwareMap.get(DcMotor.class, "mantis");
            telemetry.addLine("Mantis init successful");
            telemetry.update();
        } catch (NullPointerException e) {
            telemetry.addLine("Mantis failed to init");
            telemetry.update();
        }

        try {
            hardware.grabber = hardwareMap.get(Servo.class, "grabber");
            telemetry.addLine("Grabber init successful");
            telemetry.update();
        } catch (NullPointerException e) {
            telemetry.addLine("Grabber servo failed to init");
            telemetry.update();
        }
    }

    @Override
    public void setDirection() {
        // Set the direction of each motor
        hardware.frontLeft.setDirection(DcMotor.Direction.REVERSE); // Reverse front left motor
        hardware.frontRight.setDirection(DcMotor.Direction.FORWARD); // Forward front right motor
        hardware.backLeft.setDirection(DcMotor.Direction.REVERSE); // Reverse back left motor
        hardware.backRight.setDirection(DcMotor.Direction.FORWARD); // Forward back right motor
        hardware.lift.setDirection(DcMotor.Direction.FORWARD); // Forward lift motor
        hardware.mantis.setDirection(DcMotor.Direction.FORWARD); // Forward mantis motor
        hardware.hopper.setDirection(DcMotor.Direction.FORWARD); // Forward hopper motor
    }

    @Override
    public void movement(double vertical, double strafe, double turn) {
        // Set power to each motor based on gamepad input for movement
        hardware.frontLeft.setPower(-vertical - strafe - turn); // Calculate power for front left motor
        hardware.frontRight.setPower(-vertical + strafe + turn); // Calculate power for front right motor
        hardware.backLeft.setPower(-vertical + strafe - turn); // Calculate power for back left motor
        hardware.backRight.setPower(-vertical - strafe + turn); // Calculate power for back right motor
        whileMotorsBusy(); // Check if motors are busy and update telemetry
    }

    // Control the robot's arm based on the state and speed
    @Override
    public void arm(teleop_enum state, double speed) {
        switch(state) {
            case LIFT:
                hardware.lift.setPower(speed); // Set lift motor power
                break;
            case MANTIS:
                hardware.mantis.setPower(speed); // Set mantis motor power
                break;
            case HOPPER:
                hardware.hopper.setPower(speed); // Set hopper motor power
                break;
            default:
                // Stop all motors if no valid state is provided
                hardware.lift.setPower(0);
                hardware.mantis.setPower(0);
                hardware.hopper.setPower(0);
                break;
        }
    }

    // Control the gripper's position
    @Override
    public void gripper(int pos) {
        hardware.grabber.setPosition(pos); // Set the position of the grabber servo
    }

    @Override
    public void telemetry() {
        // Provide feedback about the robot's state
        telemetry.addLine("Robot started"); // Notify that the robot has started
        telemetry.update(); // Update the telemetry display
    }

    // Check if motors are busy and display telemetry
    @Override
    public boolean whileMotorsBusy() {
        // Check if all motors are busy
        while (opModeIsActive() &&
                hardware.frontLeft.isBusy() &&
                hardware.frontRight.isBusy() &&
                hardware.backLeft.isBusy() &&
                hardware.backRight.isBusy()) {
            // Update telemetry with motor powers and positions
            telemetry.addData("Front Left Power", hardware.frontLeft.getPower());
            telemetry.addData("Front Right Power", hardware.frontRight.getPower());
            telemetry.addData("Back Left Power", hardware.backLeft.getPower());
            telemetry.addData("Back Right Power", hardware.backRight.getPower());
            telemetry.addData("Lift Power", hardware.lift.getPower());
            telemetry.addData("Mantis Power", hardware.mantis.getPower());
            telemetry.addData("Hopper Arm Power", hardware.hopper.getPower());
            telemetry.addData("Grabber Position", hardware.grabber.getPosition());
            telemetry.update(); // Update the telemetry display
            return true; // Return true while motors are busy
        }
        return false; // Return false when motors are not busy
    }

    //TODO test arms
    // Method for controlling final movement with reduced speeds
    private void finalMovement() {
        double reduction = 0.8; // Default speed reduction
        double turnReduction = 0.55; // Default turning speed reduction

        // Adjust speeds based on button presses
        if (gamepad1.a) {
            // Slow mode
            reduction = 0.4;
            turnReduction = 0.35;
        } else if (gamepad1.b) {
            // Fast mode
            reduction = 1;
            turnReduction = 1;
        } else if ((gamepad1.left_stick_button) || (gamepad1.right_stick_button)) {
            // Stop mode
            reduction = 0.0;
            turnReduction = 0.0;
        }

        // Apply movement to motors based on gamepad input
        double vertical = reduction * gamepad1.left_stick_y; // Vertical movement
        double turn = -reduction * gamepad1.left_stick_x; // Turning movement
        double strafe = -turnReduction * gamepad1.right_stick_x; // Strafe movement
        movement(vertical, strafe, turn); // Call movement method with calculated powers
    }

    // Method for controlling the arm based on gamepad input
    private void finalArm() {
        teleop_enum state = null; // Initialize state
        double armSpeed = 0; // Initialize arm speed
        // Determine arm state and speed based on gamepad input
        if (Math.abs(gamepad2.left_stick_y) > 0) {
            state = teleop_enum.MANTIS; // Set state to MANTIS
            armSpeed = gamepad2.left_stick_y; // Use left stick Y for speed
        } else if (Math.abs(gamepad2.left_stick_x) > 0) {
            state = teleop_enum.HOPPER; // Set state to HOPPER
            armSpeed = gamepad2.left_stick_x; // Use left stick X for speed
        } else if (Math.abs(gamepad2.right_stick_y) > 0) {
            state = teleop_enum.LIFT; // Set state to LIFT
            armSpeed = gamepad2.right_stick_y; // Use right stick Y for speed
        }
        arm(state, armSpeed); // Call arm method with determined state and speed
    }

    // Method for controlling the gripper based on gamepad input
    private void finalGrabber() {
        int close = -200; // Position to close the gripper
        int open = 200; // Position to open the gripper
        // Control gripper based on button presses
        if (gamepad2.x) {
            gripper(close); // Close gripper
        } else if (gamepad2.y) {
            gripper(open); // Open gripper
        }
    }

    @Override
    public void runOpMode() {
        // Initialize the OpMode
        initialize(); // Initialize hardware
        setDirection(); // Set motor directions
        telemetry(); // Send initial telemetry data

        waitForStart(); // Wait for the start signal

        // Main loop for controlling the robot during teleop
        while (opModeIsActive()) {
            finalMovement(); // Control robot movement
            finalArm(); // Control robot arm
            finalGrabber(); // Control gripper
        }
    }
}
