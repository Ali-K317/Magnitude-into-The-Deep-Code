//package org.firstinspires.ftc.teamcode.Final.Final.Auto;
//
//import android.graphics.Color;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.ColorSensor;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.teamcode.hardware;
//import org.firstinspires.ftc.teamcode.mainEnum;
//
//@Autonomous(name = "Into the Deep Autonomous", group = "Final")
//public class IntoTheDeepAuto extends LinearOpMode implements AutoInterface {
//    hardware hardware = new hardware();
//    calculations calculations = new calculations();
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        // Initializes
//        armInit();
//        wheelInit();
//        sensorInit();
//        servoInit();
//
//        // Direction
//        setArmDirection();
//        setWheelDirection();
//
//        // Brakes
//        armBrake();
//        wheelBrake();
//
//        // Telemetry
//        telemetry();
//        calculations.timer.reset();
//
//        waitForStart();
//        while (opModeIsActive()) {
//            telemetry.addLine("Code Starting");
//            int colorThreshold = 300;
//            int red = hardware.colorSensor.red();
//            int blue = hardware.colorSensor.blue();
//            int green = hardware.colorSensor.green();
//
//
//            while(red > blue && red >green && red > colorThreshold){
//                colorTemeletry();
//                setWheelSpeed(mainEnum.BACKWARD, 0.1);
//            }
//            setWheelSpeed(mainEnum.BACKWARD, 0);
//        }
//    }
//
//    private void setSample(){
//        //Go forward until the cage is one inch away
//        while(hardware.distanceSensorBack.getDistance(DistanceUnit.INCH) < 1){
//            setWheelSpeed(mainEnum.FORWARD, calculations.driveSpeed);
//        }
//
//        //Strafe left until the cage is not detected
//        while(hardware.distanceSensorBack.getDistance(DistanceUnit.INCH) > 0){
//            setWheelSpeed(mainEnum.STRAFE_LEFT, calculations.driveSpeed);
//        }
//
//        //Rotate 90 degrees
//        while(calculations.timer.seconds() < (calculations.timeToRotate360/4)){
//            setWheelSpeed(mainEnum.TURN_RIGHT, calculations.turnSpeed);
//        }
//
//        //Strafe left for 3 seconds
//        while(calculations.timer.seconds() <= 3){
//            setWheelSpeed(mainEnum.STRAFE_LEFT, calculations.driveSpeed);
//        }
//
//        //Lift arm and put sample on
//        while(calculations.timer.seconds() < 1){
//            setArmSpeed(mainEnum.LIFT, calculations.liftUp);
//        }
//        setArmSpeed(mainEnum.LIFT, calculations.liftHold);
//        while(calculations.timer.seconds() < 1){
//            setWheelSpeed(mainEnum.FORWARD, calculations.driveSpeed);
//        }
//        setArmSpeed(mainEnum.LIFT, calculations.liftDown);
//    }
//
//    // Initialization functions
//    @Override
//    public void armInit() {
//        hardware.mantis1 = hardwareMap.get(DcMotor.class, "mantis");
//        hardware.lift = hardwareMap.get(DcMotor.class, "lift");
//        hardware.hopper = hardwareMap.get(DcMotor.class, "hopper");
//    }
//
//    @Override
//    public void wheelInit() {
//        hardware.frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
//        hardware.frontRight = hardwareMap.get(DcMotor.class, "frontRight");
//        hardware.backLeft = hardwareMap.get(DcMotor.class, "backLeft");
//        hardware.backRight = hardwareMap.get(DcMotor.class, "backRight");
//    }
//
//    @Override
//    public void sensorInit() {
//        hardware.colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
//        hardware.distanceSensorBack = hardwareMap.get(DistanceSensor.class, "distanceSensorBack");
//        hardware.distanceSensorLeft = hardwareMap.get(DistanceSensor.class, "distanceSensorLeft");
//        hardware.distanceSensorRight = hardwareMap.get(DistanceSensor.class, "distanceSensorRight");
//    }
//
//    @Override
//    public void servoInit() {
//        hardware.wrist = hardwareMap.get(DcMotor.class, "wrist");
//        hardware.door = hardwareMap.get(Servo.class, "door");
//        hardware.topGrabber = hardwareMap.get(CRServo.class, "topGrabber");
//        hardware.bottomGrabber = hardwareMap.get(CRServo.class, "bottomGrabber");
//    }
//
//    // Direction setup
//    @Override
//    public void setArmDirection() {
//        hardware.lift.setDirection(DcMotor.Direction.REVERSE);
//        hardware.mantis.setDirection(DcMotor.Direction.REVERSE);
//        hardware.hopper.setDirection(DcMotor.Direction.FORWARD);
//    }
//
//    @Override
//    public void setWheelDirection() {
//        hardware.frontLeft.setDirection(DcMotor.Direction.REVERSE);
//        hardware.frontRight.setDirection(DcMotor.Direction.FORWARD);
//        hardware.backLeft.setDirection(DcMotor.Direction.REVERSE);
//        hardware.backRight.setDirection(DcMotor.Direction.FORWARD);
//    }
//
//
//    private void colorTemeletry(){
//        int red = hardware.colorSensor.red();
//        int blue = hardware.colorSensor.blue();
//        int green = hardware.colorSensor.green();
//
//        telemetry.addData("Red", red);
//        telemetry.addData("Blue", blue);
//        telemetry.addData("Green", green);
//        telemetry.update();
//    }
//    // Braking functions
//    @Override
//    public void armBrake() {
//        hardware.lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        hardware.mantis.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        hardware.hopper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//    }
//
//    @Override
//    public void wheelBrake() {
//        hardware.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        hardware.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        hardware.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        hardware.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//    }
//
//    @Override
//    public void setWheelSpeed(mainEnum state, double speed) {
//        switch (state) {
//            case FORWARD:
//                hardware.frontLeft.setPower(speed);
//                hardware.frontRight.setPower(speed);
//                hardware.backLeft.setPower(speed);
//                hardware.backRight.setPower(speed);
//                break;
//
//            case BACKWARD:
//                hardware.frontLeft.setPower(-speed);
//                hardware.frontRight.setPower(-speed);
//                hardware.backLeft.setPower(-speed);
//                hardware.backRight.setPower(-speed);
//                break;
//
//            case STRAFE_LEFT:
//                hardware.frontLeft.setPower(-speed);
//                hardware.frontRight.setPower(speed);
//                hardware.backLeft.setPower(speed);
//                hardware.backRight.setPower(-speed);
//                break;
//
//            case STRAFE_RIGHT:
//                hardware.frontLeft.setPower(speed);
//                hardware.frontRight.setPower(-speed);
//                hardware.backLeft.setPower(-speed);
//                hardware.backRight.setPower(speed);
//                break;
//
//            case TURN_LEFT:
//                hardware.frontLeft.setPower(-speed);
//                hardware.frontRight.setPower(speed);
//                hardware.backLeft.setPower(-speed);
//                hardware.backRight.setPower(speed);
//                break;
//
//            case TURN_RIGHT:
//                hardware.frontLeft.setPower(speed);
//                hardware.frontRight.setPower(-speed);
//                hardware.backLeft.setPower(speed);
//                hardware.backRight.setPower(-speed);
//                break;
//
//            default:
//                // Handle invalid direction case
//                hardware.frontLeft.setPower(0);
//                hardware.frontRight.setPower(0);
//                hardware.backLeft.setPower(0);
//                hardware.backRight.setPower(0);
//                break;
//        }
//    }
//
//    @Override
//    public void setArmSpeed(mainEnum state,double speed) {
//        switch (state) {
//            case LIFT:
//                hardware.lift.setPower(speed); // Set lift motor power
//                break;
//            case MANTIS:
//                hardware.mantis.setPower(speed); // Set mantis motor power
//                break;
//            case HOPPER:
//                hardware.hopper.setPower(speed); // Set hopper motor power
//                break;
//            default:
//                hardware.mantis.setPower(calculations.mantisHold);
//                hardware.lift.setPower(calculations.liftHold);
//                hardware.hopper.setPower(calculations.hopperHold);
//                break;
//        }
//    }
//
//    @Override
//    public void setClawSpeed(mainEnum state, double speedTopGrabber, double speedBottomGrabber, double wristSpeed, int doorPos) {
//        switch(state){
//            case GRABBER:
//                hardware.topGrabber.setPower(-speedTopGrabber);
//                hardware.bottomGrabber.setPower(speedBottomGrabber);
//                break;
//            case WRIST:
//                hardware.wrist.setPower(wristSpeed);
//                break;
//            case DOOR:
//                hardware.door.setPosition(doorPos);
//                break;
//            default:
//                hardware.wrist.setPower(calculations.wristHold);
//                hardware.topGrabber.setPower(calculations.grabberHold);
//                hardware.bottomGrabber.setPower(calculations.grabberHold);
//                break;
//        }
//    }
//
//    // Miscellaneous
//    @Override
//    public void telemetry() {
//        telemetry.addLine("Code is running");
//        telemetry.update();
//    }
//
//    @Override
//    public void whileMotorsBusy() {
//        telemetry.addLine("Code is running");
//    }
//
//}
