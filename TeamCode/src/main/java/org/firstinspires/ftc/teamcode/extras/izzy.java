package org.firstinspires.ftc.teamcode.extras;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//@Autonomous
public class izzy extends LinearOpMode {
    // Motors
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
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
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * GEAR_RATIO ) / (Math.PI*WHEEL_DIAMETER_INCHES);

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



    private void initialize () {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight" );
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        backRight = hardwareMap.get(DcMotor.class, "backLeft");

    }
    
    private void rightStrafe (int inches) {

        frontLeft.setTargetPosition(inches);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

    }
}
