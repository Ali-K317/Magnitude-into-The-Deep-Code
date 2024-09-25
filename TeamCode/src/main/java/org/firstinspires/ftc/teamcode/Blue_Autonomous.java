package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.function.IntToDoubleFunction;

@Autonomous(name = "Blue Autonomous Into The Deep")
public class Blue_Autonomous extends LinearOpMode {
    // Motors
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private DcMotor jointOne = null;
    private DcMotor jointTwo = null;

    // Servos
    private Servo claw = null;
    private Servo horizontalWrist = null;
    private Servo verticalWrist = null;

    // Sensors
    //private DistanceSensor distanceSensor;
    private ColorSensor colorSensor;

    // Timer
    private ElapsedTime runtime = new ElapsedTime();

    // Wheel and wheel motor measurement
    private static final double GEAR_RATIO = 1.0/1.0;
    private static final double COUNTS_PER_MOTOR_REV = 28;
    private static final double WHEEL_DIAMETER_INCHES = 3.5;
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * GEAR_RATIO) / (Math.PI*WHEEL_DIAMETER_INCHES);

    //Robot speed
    private final double turnSpeed = 0.2;
    private final double driveSpeed = 0.3;

    //Robot body measurements
    private final double robotLength = 16.0;
    private final double robotWidth = 16.0;
    private final double robotHeight = 16.0;
    private final double halfRL = robotLength / 2;
    private final double halfRW = robotWidth / 2;
    private final double halfRH = robotHeight / 2;


    int red = colorSensor.red();
    int green = colorSensor.green();
    int blue = colorSensor.blue();
    boolean yellow = false;

    // Determine alliance and enemy colors
    double allianceColor = blue;
    int enemyColor = red;

    //double distance = distanceSensor.getDistance(DistanceUnit.INCH);



    // Initializes motors, servos, and sensors
    private void initialize() {
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        //distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        jointOne = hardwareMap.get(DcMotor.class, "jointOne");
        jointTwo = hardwareMap.get(DcMotor.class, "jointTwo");

        claw = hardwareMap.get(Servo.class, "claw");
        horizontalWrist = hardwareMap.get(Servo.class, "horizontalWrist");
        verticalWrist = hardwareMap.get(Servo.class, "verticalWrist");
    }

    // Sets the motors direction when powered with a positive number
    private void setDirection() {

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        jointOne.setDirection(DcMotor.Direction.REVERSE);
        jointTwo.setDirection(DcMotor.Direction.REVERSE);
    }

    // Resets all motors' encoder counts
    private void stopAndResetEncoders() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        jointOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        jointTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sleep(50);
    }

    // Sets motors' target position
    private void setPosition(int targetEncoderCount) {
        frontLeft.setTargetPosition(targetEncoderCount);
        frontRight.setTargetPosition(targetEncoderCount);
        backLeft.setTargetPosition(targetEncoderCount);
        backRight.setTargetPosition(targetEncoderCount);
    }

    // Runs motors to their target position
    private void runToPosition() {
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // Sets motors to brake when not in use
    private void setBrake() {
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        jointOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        jointTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // Updates telemetry data
    private void setTelemetry() {
        int threshold = 200;
        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();
        int allianceColor = red;
        int enemyColor = blue;
        boolean yellow;
        if ((blue+1000<red)&&(blue+1000<green)){ yellow = true;}
        else{ yellow = false;}

        telemetry.addData("Red", red);
        telemetry.addData("Blue", blue);
        telemetry.addData("Green", green);
        telemetry.addData("Yellow", yellow ? "True" : "False");
        telemetry.addData("Red winning", allianceColor < 200 ? "False" : "True");
        telemetry.update();
    }

    // Moves the robot forward
    private void forward(double inches, double speed) {
        int targetEncoderCount = (int) (inches * COUNTS_PER_INCH) + (int) (robotLength * COUNTS_PER_INCH);
        setPosition(targetEncoderCount);
        runToPosition();
        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);

        while (opModeIsActive() && (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy())) {
            setTelemetry();
        }
        stopAndResetEncoders();
        stopRobot(1.0);
    }

    // Moves the robot backward
    private void backward(double inches, double speed) {
        int targetEncoderCount = -((int) (inches * COUNTS_PER_INCH) + (int) (robotLength * COUNTS_PER_INCH));
        setPosition(targetEncoderCount);
        runToPosition();
        frontLeft.setPower(-speed);
        frontRight.setPower(-speed);
        backLeft.setPower(-speed);
        backRight.setPower(-speed);

        while (opModeIsActive() && (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy())) {
            setTelemetry();
        }
        stopAndResetEncoders();
        stopRobot(1.0);
    }

    // Strafe left movement
    private void strafeLeft(double inches, double speed) {
        int targetEncoderCount = (int) (inches * COUNTS_PER_INCH) + (int) (robotLength * COUNTS_PER_INCH);
        frontLeft.setTargetPosition(-targetEncoderCount);
        frontRight.setTargetPosition(targetEncoderCount);
        backLeft.setTargetPosition(targetEncoderCount);
        backRight.setTargetPosition(-targetEncoderCount);
        runToPosition();
        frontLeft.setPower(-speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(-speed);

        while (opModeIsActive() && (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy())) {

        }
        stopAndResetEncoders();
        stopRobot(1.0);
    }

    // Strafe right movement
    private void strafeRight(double inches, double speed) {
        int targetEncoderCount = (int) (inches * COUNTS_PER_INCH) + (int) (robotLength * COUNTS_PER_INCH);
        frontLeft.setTargetPosition(targetEncoderCount);
        frontRight.setTargetPosition(-targetEncoderCount);
        backLeft.setTargetPosition(-targetEncoderCount);
        backRight.setTargetPosition(targetEncoderCount);
        runToPosition();
        frontLeft.setPower(speed);
        frontRight.setPower(-speed);
        backLeft.setPower(-speed);
        backRight.setPower(speed);

        while (opModeIsActive() && (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy())) {

        }
        stopAndResetEncoders();
        stopRobot(1.0);
    }

    // Turn right by specified degrees
    private void turnRight(double degrees, double speed) {
        double turningRadius = 13.0; // TODO: Find distance from center to wheel
        //uses 2piR^2 to calculate the robots circumstance of turning
        double turnCircumference = 2 * Math.PI * turningRadius;
        //calculates how much of the robots circumstance is needed per degree
        double distancePerWheel = (degrees / 360) * turnCircumference;
        //how many encoder ticks are required for the robot to turn n degrees
        int targetEncoderCount = (int) (distancePerWheel * COUNTS_PER_INCH);

        frontLeft.setTargetPosition(targetEncoderCount);
        backLeft.setTargetPosition(targetEncoderCount);
        frontRight.setTargetPosition(-targetEncoderCount);
        backRight.setTargetPosition(-targetEncoderCount);
        runToPosition();
        frontLeft.setPower(speed);
        backLeft.setPower(speed);
        frontRight.setPower(-speed);
        backRight.setPower(-speed);

        while (opModeIsActive() && (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy())) {

        }
        stopAndResetEncoders();
        stopRobot(1.0);
    }

    // Turn left by specified degrees
    private void turnLeft(double degrees, double speed) {
        //robots radius from center of robot to wheel
        double turningRadius = 13.0; // TODO: Find distance from center to wheel
        //uses 2piR^2 to calculate the robots circumstance of turning
        double turnCircumference = 2 * Math.PI * turningRadius;
        //calculates how much of the robots circumstance is needed per degree
        double distancePerWheel = (degrees / 360) * turnCircumference;
        //how many encoder ticks are required for the robot to turn n degrees
        int targetEncoderCount = (int) (distancePerWheel * COUNTS_PER_INCH);

        frontLeft.setTargetPosition(-targetEncoderCount);
        backLeft.setTargetPosition(-targetEncoderCount);
        frontRight.setTargetPosition(targetEncoderCount);
        backRight.setTargetPosition(targetEncoderCount);
        runToPosition();
        frontLeft.setPower(-speed);
        backLeft.setPower(-speed);
        frontRight.setPower(speed);
        backRight.setPower(speed);

        while (opModeIsActive() && (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy())) {

        }
        stopAndResetEncoders();
        stopRobot(1.0);
    }

    // Opens the claw to a specific position
    private void openClaw(double openPosition) {
        claw.setPosition(openPosition);
    }

    // Closes the claw to a specific position
    private void closeClaw(double closePosition) {
        claw.setPosition(closePosition);
    }

    // Stops the robot by setting power to 0
    private void stopRobot(double time) {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        while(opModeIsActive() && runtime.seconds() < time){

        }
        stopAndResetEncoders();
    }


    private void startPosition(){
        //TODO use distance sensors to set your initial starting position
        forward(5.0, 0.3);
        double turnAngle = 90;
        // Move forward while alliance color is weaker than enemy color
        turnRight(turnAngle,turnSpeed);
        stopRobot(2.0);
        backward(4.0, driveSpeed);
        stopRobot(1.0);
    }
    private void detectYellow(){
        if ((blue < red) && (blue  < green)) yellow = true;
        else yellow = false;

        while (!yellow && opModeIsActive()) {
            red = colorSensor.red();
            green = colorSensor.green();
            blue = colorSensor.blue();
            // Check if yellow is detected
            if ((blue < red) && (blue < green)) {
                yellow = true;
            } else {
                // Move forward if yellow is not detected
                goToBox();
            }
        }
    }
    private void goToBox(){
        //TODO make a function that goes to the box and drops the block
        strafeLeft(0.1,driveSpeed);
    }
    private void goBackToStart(){
        //TODO make a function that goes make to its initial position
        //QUESTION maybe do the opposite of goToBox()?
        forward(1.0, 0.1);
    }





    @Override
    public void runOpMode() {
        initialize();
        setBrake();
        stopAndResetEncoders();
        setDirection();

        waitForStart();
        while (opModeIsActive()) {
            startPosition();
            detectYellow();
            goBackToStart();
            break;
        }

    }
}