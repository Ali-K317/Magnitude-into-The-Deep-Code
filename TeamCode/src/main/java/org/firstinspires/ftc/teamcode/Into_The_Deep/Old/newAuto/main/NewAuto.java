//package org.firstinspires.ftc.teamcode.autonomous.newAuto.main;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.ColorSensor;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.teamcode.Into_The_Deep.mainEnum;
//import org.firstinspires.ftc.teamcode.Into_The_Deep.hardware;
//
//@Autonomous(name = "New Autonomous", group = "Autonomous")
//public class NewAuto extends LinearOpMode implements newAuto_interface {
//    final hardware hardware = new hardware();
//    final calculations calculations = new calculations();
//    final int open = 200;
//    final int closed = 0;
//
//    final int tickPer10cm = 100;
//    final int tickPerCm = tickPer10cm/10;
//
//    // Initialize motors and hardware components
//    @Override
//    public void initialize() {
//        try {
//            //DcMotor
//            hardware.frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
//            hardware.frontRight = hardwareMap.get(DcMotor.class, "frontRight");
//            hardware.backLeft = hardwareMap.get(DcMotor.class, "backLeft");
//            hardware.backRight = hardwareMap.get(DcMotor.class, "backRight");
//
//            hardware.mantis = hardwareMap.get(DcMotor.class, "mantis");
//            hardware.lift = hardwareMap.get(DcMotor.class, "lift");
//            hardware.hopper = hardwareMap.get(DcMotor.class, "hopper");
//
//            //Servos
//            hardware.grabber = hardwareMap.get(CRServo.class, "grabber");
//            hardware.door = hardwareMap.get(Servo.class, "door");
//            hardware.wrist = hardwareMap.get(Servo.class, "wrist");
//
//            //Sensors
//            hardware.colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
//            hardware.distanceSensorLeft = hardwareMap.get(DistanceSensor.class, "distanceSensorLeft");
//            hardware.distanceSensorBack = hardwareMap.get(DistanceSensor.class, "distanceSensorBack");
//            hardware.distanceSensorRight = hardwareMap.get(DistanceSensor.class, "distanceSensorRight");
//
//            hardware.checkMotorInit();
//        } catch (NullPointerException e) {
//            telemetry.addLine("Initialization error: " + e.getMessage());
//            telemetry.update();
//        } catch (Exception e) {
//            telemetry.addLine("An error occurred during initialization: " + e.getMessage());
//            telemetry.update();
//        }
//    }
//
//    // Sets the robot telemetry data
//    @Override
//    public void telemetry() {
//        telemetry.addLine("Code is running");
//        String randomPun = hardware.puns[calculations.random.nextInt(hardware.puns.length)];
//        // Provide feedback about the robot's state
//        while (opModeInInit()) {
//            telemetry.addLine("=== Robot Initialization ===");
//            telemetry.addLine("Status: Initializing");
//            telemetry.addLine("Press START when ready");
//            telemetry.addData("Pun of the day", randomPun);
//            telemetry.update();
//        }
//        whileMotorsBusy();
//    }
//
//    // While the motor is busy, run telemetry
//    public void whileMotorsBusy() {
//        telemetry.addLine("Code is running");
//        if (hardware.frontLeft.isBusy()) {
//            telemetry.addLine("=== Wheel ===");
//            telemetry.addData("Front left motor position", hardware.frontLeft.getCurrentPosition());
//            telemetry.addData("Front right motor position", hardware.frontRight.getCurrentPosition());
//            telemetry.addData("Back left motor position", hardware.backLeft.getCurrentPosition());
//            telemetry.addData("Back right motor position", hardware.backRight.getCurrentPosition());
//        } else if (hardware.lift.isBusy()) {
//            telemetry.addLine("=== Lift ===");
//            telemetry.addData("Lift motor position", hardware.lift.getCurrentPosition());
//        } else if (hardware.mantis.isBusy()) {
//            telemetry.addLine("=== Mantis ===");
//            telemetry.addData("Mantis Motor Position", hardware.mantis.getCurrentPosition());
//        } else if (hardware.hopper.isBusy()) {
//            telemetry.addLine("=== Hopper Arm ===");
//            telemetry.addData("Hopper Motor Position", hardware.hopper.getCurrentPosition());
//        } else if (hardware.grabber.getPower() != 0) {
//            telemetry.addLine("=== Grabber ===");
//            telemetry.addData("Grabber Position", hardware.grabber.getPower());
//        }
//        telemetry.update();
//    }
//
//    // Sets which direction the motor goes in when plugging in a positive power
//    @Override
//    public void setDirection() {
//        hardware.frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        hardware.frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
//        hardware.backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        hardware.backRight.setDirection(DcMotorSimple.Direction.FORWARD);
//
//        hardware.lift.setDirection(DcMotorSimple.Direction.FORWARD);
//        hardware.mantis.setDirection(DcMotorSimple.Direction.FORWARD);
//        hardware.hopper.setDirection(DcMotorSimple.Direction.FORWARD);
//    }
//
//    @Override
//    public void setBrakes(){
//        hardware.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        hardware.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        hardware.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        hardware.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        hardware.lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        hardware.mantis.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        hardware.hopper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//    }
//
//    // Sets the motor to run to a position set by the number of encoder ticks
//    @Override
//    public void runToPosition(DcMotor motor) {
//        if (motor == null) {
//            telemetry.addLine("Cannot reset encoder; motor is null.");
//            telemetry.update();
//            return; // Exit the method if motor is null
//        }
//
//        try {
//            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            telemetry.addLine("Resetting encoder for motors");
//        } catch (Exception e) {
//            telemetry.addLine("Error resetting encoder for motor");
//            telemetry.update();
//        }
//    }
//
//    @Override
//    public void wheelMotorToPosition() {
//        if (hardware.frontLeft.isBusy() || hardware.frontRight.isBusy() || hardware.backLeft.isBusy() || hardware.backRight.isBusy()) {
//            runToPosition(hardware.frontLeft);
//            runToPosition(hardware.frontRight);
//            runToPosition(hardware.backLeft);
//            runToPosition(hardware.backRight);
//        }
//        if(hardware.lift.isBusy() || hardware.mantis.isBusy() || hardware.hopper.isBusy()){
//            runToPosition(hardware.lift);
//            runToPosition(hardware.mantis);
//            runToPosition(hardware.hopper);
//        }
//    }
//
//    // Helper method to reset motor encoder
//    @Override
//    public void resetEncoder(DcMotor motor) {
//        if (motor == null) {
//            telemetry.addLine("Cannot reset encoder; motor is null.");
//            telemetry.update();
//            return; // Exit the method if motor is null
//        }
//        try {
//            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            telemetry.addLine("Resetting encoder for motors");
//        } catch (Exception e) {
//            telemetry.addLine("Error resetting encoder for motor");
//            telemetry.update();
//        }
//    }
//
//    // Reset the encoders for all motors
//    @Override
//    public void resetMotorEncoders() {
//        resetEncoder(hardware.frontLeft);
//        resetEncoder(hardware.frontRight);
//        resetEncoder(hardware.backLeft);
//        resetEncoder(hardware.backRight);
//        resetEncoder(hardware.lift);
//        resetEncoder(hardware.mantis);
//        resetEncoder(hardware.hopper);
//    }
//
//    // Sets the robot's encoder position to go to
//    public void setPosition(int frontLeftPos, int frontRightPos, int backLeftPos, int backRightPos) {
//        hardware.frontLeft.setTargetPosition(frontLeftPos);
//        hardware.frontRight.setTargetPosition(frontRightPos);
//        hardware.backLeft.setTargetPosition(backLeftPos);
//        hardware.backRight.setTargetPosition(backRightPos);
//    }
//
//    // Sets the motor speeds
//    public void setWheelSpeed(double frontLeftSpeed, double frontRightSpeed, double backRightSpeed, double backLeftSpeed) {
//        hardware.frontLeft.setPower(frontLeftSpeed);
//        hardware.frontRight.setPower(frontRightSpeed);
//        hardware.backLeft.setPower(backLeftSpeed);
//        hardware.backRight.setPower(backRightSpeed);
//    }
//
//    // The base for all other movement functions
//    @Override
//    public void base(int targetPosFL, int targetPosFR, int targetPosBL, int targetPosBR, double speedFL, double speedFR, double speedBL, double speedBR) {
//        if(opModeIsActive()){
//            setPosition(targetPosFL, targetPosFR, targetPosBL, targetPosBR);
//            wheelMotorToPosition();
//            while(hardware.frontLeft.getCurrentPosition() < targetPosFL || hardware.frontRight.getCurrentPosition() < targetPosFR || hardware.backLeft.getCurrentPosition() < targetPosBL || hardware.backRight.getCurrentPosition() < targetPosBR){
//                setWheelSpeed(speedFL, speedFR, speedBL, speedBR);
//                while (hardware.frontLeft.isBusy()) {
//                    whileMotorsBusy();
//                }
//            }
//            setWheelSpeed(0, 0, 0, 0);
//            resetMotorEncoders();
//            sleep(250);
//        }
//    }
//
//
//    // Movement logic based on state
//    @Override
//    public void movement(mainEnum state, double tick, double rotation, double speed) {
//        int targetPos = (int) (tick * calculations.tick_per_inch);
//        int targetRotation = (int) (rotation * calculations.tick_per_360);
//        switch (state) {
//            case FORWARD:
//                base(targetPos, targetPos, targetPos, targetPos, speed, speed, speed, speed);
//                break;
//            case BACKWARD:
//                base(-targetPos, -targetPos, -targetPos, -targetPos, -speed, -speed, -speed, -speed);
//                break;
//            case STRAFE_LEFT:
//                base(-targetPos, targetPos, targetPos, -targetPos, -speed, speed, speed, -speed);
//                break;
//            case STRAFE_RIGHT:
//                base(targetPos, -targetPos, -targetPos, targetPos, speed, -speed, -speed, speed);
//                break;
//            case TURN_RIGHT:
//                 base(targetRotation, targetRotation, -targetRotation, -targetRotation, speed, speed, -speed, -speed);
//                 break;
//            case TURN_LEFT:
//                 base(-targetRotation, -targetRotation, targetRotation, targetRotation, -speed, -speed, speed, speed);
//                 break;
//        }
//    }
//
//    @Override
//    public void arm(mainEnum motor, double inch, double speed) {
//        int targetPos;
//        switch (motor) {
//            case MANTIS:
//                targetPos = (int) (calculations.MANTIS_TICK_PER_INCH * inch);
//                hardware.mantis.setTargetPosition(targetPos);
//                hardware.mantis.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                while(hardware.mantis.getCurrentPosition() < targetPos) {
//                    hardware.mantis.setPower(speed);
//                    whileMotorsBusy();
//                }
//                hardware.mantis.setPower(0.1);
//                hardware.mantis.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                sleep(250);
//                break;
//            case LIFT:
//                targetPos = (int) (calculations.LIFT_TICK_PER_INCH * inch);
//                hardware.lift.setTargetPosition(targetPos);
//                hardware.mantis.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                while(hardware.lift.getCurrentPosition() < targetPos) {
//                    hardware.lift.setPower(speed);
//                    whileMotorsBusy();
//                }
//                hardware.lift.setPower(0.0);
//                hardware.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                sleep(250);
//                break;
//            case HOPPER:
//                targetPos = (int) (calculations.HOPPER_TICK_PER_INCH * inch);
//                hardware.hopper.setTargetPosition(targetPos);
//                hardware.hopper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                while(hardware.hopper.getCurrentPosition() < targetPos) {
//                    hardware.hopper.setPower(speed);
//                    whileMotorsBusy();
//                }
//                hardware.hopper.setPower(0.0);
//                hardware.hopper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                sleep(250);
//                break;
//        }
//    }
//    @Override
//    public void grabber(mainEnum state){
//        double power = 0;
//        int pos = 0;
//        switch (state){
//            case RELEASE:
//                power = -1;
//                hardware.grabber.setPower(power);
//                break;
//            case OPEN:
//                power = 1;
//                hardware.door.setPosition(power);
//                break;
//            case CLOSE:
//                power = -1;
//                hardware.door.setPosition(power);
//                break;
//            case GRAB:
//                power = 1;
//                hardware.grabber.setPower(power);
//                break;
//            case WRIST_UP:
//                pos = 200;
//                hardware.wrist.setPosition(pos);
//                break;
//            case WRIST_DOWN:
//                pos = 0;
//                hardware.wrist.setPosition(pos);
//                break;
//        }
//    }
//
//
//
//    @Override
//    public boolean detectYellow(int blue, int red, int green) {
//        return (blue < red) && (blue < green);
//    }
//
//    // Moves to an area based on a state given
//    @Override
//    public void moveTo(mainEnum state) {
//        switch (state) {
//            case START_POSITION:
//                //Move forward turning radius plus an inch for error
//                movement(mainEnum.FORWARD, 5, 0, calculations.DRIVE_SPEED);
//
//                //Turn right 90degrees so that the color sensor is closest to the wall
//                movement(mainEnum.TURN_RIGHT,0, 90, calculations.DRIVE_SPEED);
//
//                //Strafe right until the distance sensor is at DISTANCE_FROM_BLOCK inches from wall
//                while (hardware.distanceSensorRight.getDistance(DistanceUnit.INCH) < calculations.DISTANCE_FROM_WALL) {
//                    movement(mainEnum.STRAFE_RIGHT, calculations.SMIDGEN, 0, calculations.DRIVE_SPEED);
//                }
//                //Move backwards a set number of inches until you're near blocks
//                movement(mainEnum.BACKWARD, calculations.DISTANCE_TO_BLOCKS, 0, calculations.DRIVE_SPEED);
//
//                //Give back data
//                telemetry.speak("Going to start position");
//                telemetry.update();
//                sleep(1000);
//                break;
//            case GO_TO_BASKET:
//                movement(mainEnum.FORWARD, 5, 0,calculations.DRIVE_SPEED);
//                telemetry.speak("Heading towards the bucket");
//                telemetry.update();
//                sleep(1000);
//                break;
//            case END_POSITION:
//                movement(mainEnum.BACKWARD, 3, 0, calculations.DRIVE_SPEED);
//                telemetry.speak("Going back to start position");
//                telemetry.update();
//                sleep(1000);
//                break;
//        }
//    }
//
//
//    // Main method to run the autonomous operation
//    @Override
//    public void runOpMode() throws InterruptedException {
//        initialize();
//        setDirection();
//        setBrakes();
//        resetMotorEncoders();
//        waitForStart();
//        while (opModeIsActive()) {
//            int red = hardware.colorSensor.red();
//            int blue = hardware.colorSensor.blue();
//            int green = hardware.colorSensor.green();
//            movement(mainEnum.FORWARD, tickPer10cm,0,calculations.DRIVE_SPEED);
//        }
//    }
//}