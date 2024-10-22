package org.firstinspires.ftc.teamcode.autonomous.oldAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//@Autonomous(name = "Blue Autonomous Into The Deep")
public class Blue_Autonomous extends LinearOpMode {
    // Motors
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor jointOne;
    private DcMotor jointTwo;

    // Servos
    private Servo claw;
    private Servo horizontalWrist;
    private Servo verticalWrist;

    // Sensors
    //private DistanceSensor distanceSensor;
    private ColorSensor colorSensor;

    // Timer
    private ElapsedTime runtime = new ElapsedTime();

    // Wheel and wheel motor measurement
    private static final double GEAR_RATIO = 1.0/1.0;
    private static final double COUNTS_PER_MOTOR_REV = 28;
    private static final double WHEEL_DIAMETER_INCHES = 3.5;
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * GEAR_RATIO ) / (Math.PI*WHEEL_DIAMETER_INCHES);

    //Robot speed
    private final double TURNSPEED = 0.2;
    private final double DRIVESPEED = 0.3;

    //Robot body measurements
    private final double ROBOTLENGTH = 16.0;
    private final double ROBOTWIDTH = 16.0;
    private final double ROBOTHEIGHT = 16.0;
    private final double HALFRL = ROBOTLENGTH / 2;
    private final double HALfRW = ROBOTWIDTH / 2;
    private final double HALFRH = ROBOTHEIGHT / 2;



    //double distance = distanceSensor.getDistance(DistanceUnit.INCH);



    // Initializes motors, servos, and sensors
    private void initialize() {
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");//gamepad1 //conf 0
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");//gamepad1 //conf 1
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");//gamepad1  //conf 2
        backRight = hardwareMap.get(DcMotor.class, "backRight");//gamepad1 //conf 3

        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");//conf 0
        //distanceSensor = hardwareMap.get(DistanceSensor.class,"distanceSensor");//conf 1

        //expansion hub
        jointOne = hardwareMap.get(DcMotor.class, "jointOne");//gamepad2 //conf 1
        jointTwo = hardwareMap.get(DcMotor.class, "jointTwo");//gamepad2 //conf 2

        claw = hardwareMap.get(Servo.class, "claw");//gamepad2 //conf 0
        horizontalWrist = hardwareMap.get(Servo.class, "horizontalWrist");//gamepad2 //conf 1
        verticalWrist = hardwareMap.get(Servo.class, "verticalWrist");//gamepad2 //conf 2
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
        int allianceColor = blue;
        int enemyColor = red;
        boolean yellow;
        if ((blue+1000<red)&&(blue+1000<green)){ yellow = true;}
        else{ yellow = false;}

        telemetry.addData("Red", red);
        telemetry.addData("Blue", blue);
        telemetry.addData("Green", green);
        telemetry.addData("Yellow", yellow ? "True" : "False");
        telemetry.addData("Blue winning", allianceColor > 200 ? "False" : "True");
        telemetry.update();
    }

    // Moves the robot forward
    private void forward(double inches, double speed) {
        int targetEncoderCount = (int) (inches * COUNTS_PER_INCH) + (int) (ROBOTLENGTH * COUNTS_PER_INCH);
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
        int targetEncoderCount = -((int) (inches * COUNTS_PER_INCH) + (int) (ROBOTLENGTH * COUNTS_PER_INCH));
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
        int targetEncoderCount = (int) (inches * COUNTS_PER_INCH) + (int) (ROBOTLENGTH * COUNTS_PER_INCH);
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
        int targetEncoderCount = (int) (inches * COUNTS_PER_INCH) + (int) (ROBOTLENGTH * COUNTS_PER_INCH);
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
        //calculates the distance the robot must travel in terms of arc length
        double archLength = (degrees/360)*turnCircumference;
        //number of revolutions to reach the arch length
        double numOfRev = archLength / (WHEEL_DIAMETER_INCHES * Math.PI);
        //number of encoder ticks to reach the arch length
        int targetEncoderCount = (int) (numOfRev * COUNTS_PER_MOTOR_REV);

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
        double turningRadius = 13.0; // Distance from center to wheel (make sure this is accurate)

        // Uses 2πr to calculate the robot's turning circumference
        double turnCircumference = 2 * Math.PI * turningRadius;

        // Calculates how much of the robot's circumference is needed per degree
        double inchPerDegree = turnCircumference / 360.0;

        // Converts that distance into encoder counts
        int targetEncoderCount = (int) (inchPerDegree * COUNTS_PER_INCH * degrees);

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
        turnRight(turnAngle,TURNSPEED);
        backward(4.0, DRIVESPEED);
    }
    private void detectYellow(){
        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();
        boolean yellow = false;

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
        strafeLeft(0.1,DRIVESPEED);
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
        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();
        boolean yellow = false;

        waitForStart();
        while (opModeIsActive()) {
//            startPosition();
//            detectYellow();
//            goBackToStart();
            turnRight(90,0.2);
            break;
        }

    }
}