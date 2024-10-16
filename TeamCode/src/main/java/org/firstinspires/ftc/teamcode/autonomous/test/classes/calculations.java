package org.firstinspires.ftc.teamcode.autonomous.test.classes;

public class calculations{
    // Wheel and wheel motor measurement
    public static final double GEAR_RATIO = 5.0/1.0;
    public static final double COUNTS_PER_MOTOR_REV = 28;
    public static final double WHEEL_DIAMETER_INCHES = 3.5;
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * GEAR_RATIO ) / (Math.PI*WHEEL_DIAMETER_INCHES);

    public static final double WHEEL_CIRCUMFERENCE_INCHES =  2 * WHEEL_DIAMETER_INCHES * Math.PI;

    //Turning radius refers to distance from center of robot to wheel and is used for turning
    public double TURNING_RADIUS = 13.0; // TODO: Find distance from center to wheel
    //uses 2piR^2 to calculate the robots circumstance of turning
    public double TURN_CIRCUMFERENCE = 2 * Math.PI * TURNING_RADIUS;

    //Finds inches in one degree of movement
    public double INCH_PER_DEGREE = TURN_CIRCUMFERENCE/360;
    //Encoder tick per inch of movement
    public double TICK_PER_INCH = COUNTS_PER_MOTOR_REV/WHEEL_CIRCUMFERENCE_INCHES;
    //Encoder degree per degree of turning
    public double TICK_PER_360 =(((TURN_CIRCUMFERENCE/WHEEL_CIRCUMFERENCE_INCHES) * (COUNTS_PER_MOTOR_REV)))*8;
    public double TICK_PER_DEGREE = TICK_PER_360/360;

    //Robot speed
    public final double TURNSPEED = 0.2;
    public final double DRIVESPEED = 0.3;

    //Robot body measurements
    public final double ROBOTLENGTH = 16.0;
    public final double ROBOTWIDTH = 16.0;
    public final double ROBOTHEIGHT = 16.0;
    public final double HALFRL = ROBOTLENGTH / 2;
    public final double HALFRW = ROBOTWIDTH / 2;
    public final double HALFRH = ROBOTHEIGHT / 2;

}
