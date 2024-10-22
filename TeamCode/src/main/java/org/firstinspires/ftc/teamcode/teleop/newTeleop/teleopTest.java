//package org.firstinspires.ftc.teamcode.teleop.newTeleop;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Gamepad;
//
//
////@TeleOp(name = "Testing", group = "Test")
//public class teleopTest extends LinearOpMode implements teleop_interface
//{
//    hardware hardware = new hardware();
//    @Override
//    public void initialize()
//    {
//        hardware.frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
//        hardware.frontRight = hardwareMap.get(DcMotor.class, "frontRight");
//        hardware.backLeft = hardwareMap.get(DcMotor.class, "backLeft");
//        hardware.backRight = hardwareMap.get(DcMotor.class, "backRight");
//    }
//    @Override
//    public void setDirection()
//    {
//        hardware.frontLeft.setDirection(DcMotor.Direction.REVERSE);
//        hardware.frontRight.setDirection(DcMotor.Direction.FORWARD);
//        hardware.backLeft.setDirection(DcMotor.Direction.REVERSE);
//        hardware.backRight.setDirection(DcMotor.Direction.FORWARD);
//    }
//    @Override
//    public void movement(double vertical, double strafe, double turn)
//    {
//        hardware.frontLeft.setPower(-vertical - strafe - turn);
//        hardware.frontRight.setPower(-vertical + strafe + turn);
//        hardware.backLeft.setPower(-vertical + strafe - turn);
//        hardware.backRight.setPower(-vertical - strafe + turn);
//
//    }
//
//    @Override
//    public void arm(teleop_enum state, double speed) {
//
//    }
//
//    @Override
//    public void gripper(int pos) {
//
//    }
//
//    private void resetEncoders()
//    {
//        try
//        {
//            hardware.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            hardware.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            hardware.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            hardware.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//            hardware.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            hardware.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            hardware.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            hardware.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        }
//        catch (Exception e)
//        {
//            telemetry.addLine("Error resetting encoder for motor");
//            telemetry.update();
//        }
//    }
//    @Override
//    public void telemetry()
//    {
//        telemetry.addData("Front left speed", hardware.frontLeft.getPower());
//        telemetry.addData("Front right speed", hardware.frontRight.getPower());
//        telemetry.addData("Back left speed", hardware.backLeft.getPower());
//        telemetry.addData("Back right speed", hardware.backRight.getPower());
//        telemetry.addData("Front left motor position", hardware.frontLeft.getCurrentPosition());
//        telemetry.addData("Front right motor position", hardware.frontRight.getCurrentPosition());
//        telemetry.addData("Back left motor position", hardware.backLeft.getCurrentPosition());
//        telemetry.addData("Back right motor position", hardware.backRight.getCurrentPosition());
//        telemetry.update();
//    }
//    @Override
//    public boolean whileMotorsBusy()
//    {
//        while (opModeIsActive() && hardware.frontLeft.isBusy() && hardware.frontRight.isBusy() && hardware.backLeft.isBusy() && hardware.backRight.isBusy()) {
//            telemetry.addData("Front left motor position", hardware.frontLeft.getCurrentPosition());
//            telemetry.addData("Front right motor position", hardware.frontRight.getCurrentPosition());
//            telemetry.addData("Back left motor position", hardware.backLeft.getCurrentPosition());
//            telemetry.addData("Back right motor position", hardware.backRight.getCurrentPosition());
//            telemetry.addData("Front Left Power", hardware.frontLeft.getPower());
//            telemetry.addData("Front Right Power", hardware.frontRight.getPower());
//            telemetry.addData("Back Left Power", hardware.backLeft.getPower());
//            telemetry.addData("Back Right Power", hardware.backRight.getPower());
//            telemetry.addData("Time Elapsed", "%.2f seconds", getRuntime());
//            telemetry.update();
//            return true;
//        }
//        return false;
//    }
//    @Override
//    public void runOpMode() throws InterruptedException
//    {
//        try
//        {
//            initialize();
//            telemetry.addLine("Init successful");
//            telemetry.update();
//        }
//        catch (NullPointerException e)
//        {
//            telemetry.addLine("One or more motors returned null");
//            telemetry.update();
//        }
//        setDirection();
//        waitForStart();
//        while(opModeIsActive())
//        {
//            telemetry();
//            double vertical = 0.4 * gamepad1.left_stick_y;
//            double strafe = -0.4 * gamepad1.left_stick_x;
//            double turn = -0.35 * gamepad1.right_stick_x;
//            if (gamepad1.a)
//            {
//                vertical = 0.2 * gamepad1.left_stick_y;
//                strafe = -0.2 * gamepad1.left_stick_x;
//                turn = -0.15 * gamepad1.right_stick_x;
//            }
//            else if (gamepad1.b)
//            {
//                vertical = 0.6 * gamepad1.left_stick_y;
//                strafe = -0.6 * gamepad1.left_stick_x;
//                turn = -0.55 * gamepad1.right_stick_x;
//            }
//            else if ((gamepad1.left_stick_button) || (gamepad1.right_stick_button))
//            {
//                vertical = 0.0;
//                strafe = 0.0;
//                turn = 0.0;
//            }
//            movement( vertical, strafe, turn);
//        }
//    }
//}