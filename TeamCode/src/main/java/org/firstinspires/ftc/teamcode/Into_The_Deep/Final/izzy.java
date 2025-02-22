package org.firstinspires.ftc.teamcode.Into_The_Deep.Final;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Into_The_Deep.hardware;

import java.util.Random;


//@Autonomous(name = "Izzy's auto", group = "Final")
public class izzy extends LinearOpMode {
    hardware hardware = new hardware();

    private final String[] puns = {
            "A robot didn’t want to have his photo taken. When he was asked why, he replied: Because I’m a photo-resistor!",
            "A robot gets arrested. He’s charged with battery.",
            "A robot man walks into a robot restaurant. A robot waiter approaches and asks him for his robot order.",
            "A robot musician’s collection of instruments will never be complete. They can never get any organs.",
            "A robot walks into a bar and says he needs to loosen up. So the bartender serves him a screwdriver.",
            "A robot walks into a bar. The bartender asks, 'What’ll ya have?' The robot says, 'Well, it’s been a long day and I need to loosen up. How about a screwdriver?'",
            "Did you hear about the writing robot who combined all the books ever written into one big novel? It’s a long story.",
            "Does R2D2 have any brothers? No. Only transisters.",
            "Hey, did you hear the story about the headless robot? According to reports, he completely lost his mind!",
            "How are A.I. blogs similar to philosophy majors? That’s easy… they’re both always trying to explain what ‘deep learning’ is!",
            "How did the robot get across the river? In a ro-boat.",
            "How did the robot’s teacher mark his book? With robo-ticks.",
            "How do you know when you’re in love with a robot? You feel a little spark.",
            "How do you reboot a robot? You kick it in its robutt.",
            "How do you use a remote control to calm down a robot dog? Press the paws button.",
            "How long is the robot alphabet? There are just two numbers – 0 and 1!",
            "How many robots does it take to screw in a light bulb? Three — one to hold the bulb, and two to turn the ladder!",
            "I bought one of those early 2000s robot dogs but have nowhere to charge it; I can’t find a place to pug it in.",
            "I finally fulfilled my dream to become a half-cyborg! It did cost me an arm and a leg, though.",
            "I got a new robot dog last week. Its name is Dogmatic.",
            "I invented a surgical robot. So far it only operates on batteries.",
            "I just got a wireless robot the other day. You could say that our relationship comes with no strings attached.",
            "I was bored, so I made a robot that distributes herbs. It helped pass the thyme.",
            "I’m not saying all factory workers are robots… All I’m saying is when they get to work they’ve returned to their factory setting.",
            "I’m starting to make a robot that has really high words per minute count. He’s a pro-to-type.",
            "If a Norwegian robot analyzed a bird, then it… Scandinavian.",
            "In a robot-only disco, one of the dancers suddenly shuts down. The doctor arrives and after a quick inspection he calms the crowd: 'Don’t worry, he just got disco-nnected.'",
            "Inventor: 'Hey, will you give me a hand?' Robot: Detaches hand, hands it to the inventor.",
            "Judge: 'So, Mr. Robot. Your neighbor accused you of stealing their electricity to power yourself. How do you plea?' Robot, the defendant: 'Guilty as charged.'",
            "My wife told me robots don’t wash themselves. So I put one in the bath and said 'That’ll shower.'",
            "Scientists have discovered a planet populated entirely by robots. They call it Mars.",
            "A robot orders a robot steak. The robot waiter asks them how they want their robot steak prepared. The robot replies, 'Weld on.'",
            "There was a giant, steel, robot who had one job, protect the city. One day when it was raining some of the screws got rusty and fell off causing one of the legs to fall off entirely. When the leg fell off it crushed the city that it was meant to protect. Oh, the iron knee!"
    };

    private final Random random = new Random();
    private final String randomPun = puns[random.nextInt(puns.length)];

    //TIME
    private ElapsedTime totalGameTime = new ElapsedTime();
    private ElapsedTime timer = new ElapsedTime();

    //SPEED
    double armSpeed = 0.5;
    double driveSpeed = 0.5;
    double turnSpeed = 0.55;

    //TIME
    double timeToRotate360 = 2.29;
    double timeToTravel1Tile = 0.73;
    double timeToLiftHopper = 1.8;
    double timeArmtoGround = 0.5;
    double timeArmtoHopper = 0.55;
    double timeToIntakeBlock = 2;
    double timeToSpitOutBlock = 1.5;

    //HATCH
    double open = 0.25;       // Open door position
    double close = 0.6;// Close door position

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        setWheelDirection();
        setArmDirection();


        while(opModeInInit()){
            timer.reset();
            telemetry.addLine(randomPun);
        }
        waitForStart();
        while (opModeIsActive()) {
            sendTelemetryData();
            armBrake("MANTIS");
            moveToHopper();
            break;
        }
    }

    //Initializes the components
    private void initialize() {
        //Wheels
        hardware.frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        hardware.frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        hardware.backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        hardware.backRight = hardwareMap.get(DcMotor.class, "backRight");

        //Arms
        hardware.mantis = hardwareMap.get(DcMotor.class, "mantis");
        hardware.lift = hardwareMap.get(DcMotor.class, "lift");
        hardware.hopper = hardwareMap.get(DcMotor.class, "hopper");

        //Claws
        hardware.bottomGrabber = hardwareMap.get(CRServo.class, "bottomGrabber");
        hardware.topGrabber = hardwareMap.get(CRServo.class, "topGrabber");
        hardware.door = hardwareMap.get(Servo.class, "door");
    }

    //Sets the directions of each component
    private void setWheelDirection() {
        hardware.frontLeft.setDirection(DcMotor.Direction.REVERSE);
        hardware.frontRight.setDirection(DcMotor.Direction.FORWARD);
        hardware.backLeft.setDirection(DcMotor.Direction.REVERSE);
        hardware.backRight.setDirection(DcMotor.Direction.FORWARD);
    }

    private void setArmDirection() {
        hardware.lift.setDirection(DcMotor.Direction.REVERSE);
        hardware.mantis.setDirection(DcMotor.Direction.REVERSE);
        hardware.hopper.setDirection(DcMotor.Direction.REVERSE);
    }

    // Function to send telemetry data
    private void sendTelemetryData() {
        // Display robot's current position, speed, and other relevant data
        telemetry.addData("Time", totalGameTime.seconds());
        telemetry.addData("Front Left Wheel Speed", hardware.frontLeft.getPower());
        telemetry.addData("Front Right Wheel Speed", hardware.frontRight.getPower());
        telemetry.addData("Back Left Wheel Speed", hardware.backLeft.getPower());
        telemetry.addData("Back Right Wheel Speed", hardware.backRight.getPower());

        telemetry.addData("Bottom Grabber Power", hardware.bottomGrabber.getPower());
        telemetry.addData("Top Grabber Power", hardware.topGrabber.getPower());
        telemetry.addData("Door Position", hardware.door.getPosition());

        // Add any additional sensor or status information
        telemetry.addData("Robot Status", "Active");

        // Update the telemetry
        telemetry.update();
    }

    //Set the components speed
    private void setWheelSpeed(double flSpeed, double frSpeed, double blSpeed, double brSpeed) {
        hardware.frontLeft.setPower(flSpeed);
        hardware.frontRight.setPower(frSpeed);
        hardware.backLeft.setPower(blSpeed * 1.6);
        hardware.backRight.setPower(brSpeed);
    }

    private void setArmSpeed(String armType, double speed) {
        switch (armType) {
            case "MANTIS":
                hardware.mantis.setPower(speed);
                break;
            case "LIFT":
                hardware.lift.setPower(speed);
                break;
            case "HOPPER":
                hardware.hopper.setPower(speed);
                break;
        }
    }

    private void armBrake(String armType) {
        switch (armType) {
            case "MANTIS":
                hardware.mantis.setPower(0.0);
                break;
            case "LIFT":
                hardware.lift.setPower(0.0);
                break;
            case "HOPPER":
                hardware.hopper.setPower(0.1);
                break;
        }
    }

    private void clawBrake(String clawType) {
        switch (clawType) {
            case "BOTTOM_GRABBER":
                hardware.bottomGrabber.setPower(0.0);
                break;
            case "TOP_GRABBER":
                hardware.topGrabber.setPower(0.0);
                break;
            case "DOOR":
                hardware.door.setPosition(hardware.door.getPosition());
                break;
        }
    }


    //Movement for the wheels, arms, and claws
    private void moveWheels(String direction, double speed) {
        switch (direction) {
            case "FORWARD":
                setWheelSpeed(speed, speed, speed, speed);
                break;
            case "BACKWARD":
                setWheelSpeed(-speed, -speed, -speed, -speed);
                break;
            case "TURN_RIGHT":
                setWheelSpeed(speed, -speed, speed, -speed);
                break;
            case "TURN_LEFT":
                setWheelSpeed(-speed, speed, -speed, speed);
                break;
        }
    }

    //Uses while loops and a time parameter to move for a set amount of time
    private void moveForTime(String direction, double time, double speed){
        switch (direction) {
            case "FORWARD":
                while(timer.seconds() <= time){
                    telemetry.addLine("Working");
                    telemetry.update();
                    moveWheels("FORWARD", speed);
                }
                timer.reset();
                break;
            case "BACKWARD":
                while(timer.seconds() <= time){
                    telemetry.addLine("Working");
                    telemetry.update();
                    moveWheels("BACKWARD", speed);
                }
                timer.reset();
                break;
            case "TURN_RIGHT":
                while(timer.seconds() <= time){
                    telemetry.addLine("Working");
                    telemetry.update();
                    moveWheels("TURN_RIGHT", speed);
                }
                timer.reset();
                break;
            case "TURN_LEFT":
                while(timer.seconds() <= time){
                    telemetry.addLine("Working");
                    telemetry.update();
                    moveWheels("TURN_LEFT", speed);
                }
                timer.reset();
                break;
        }
    }

    private void wiggle () {
        moveForTime("FORWARD", 0.02, driveSpeed);
        moveForTime("BACKWARD", 0.02, driveSpeed);
    }
    private void stopRobot(){
        moveWheels("FORWARD", 0);
    }
    //Moves the preset block to the hopper
    private void moveToHopper(){
        moveForTime("BACKWARD", timeToTravel1Tile * 2.5, driveSpeed);
        moveForTime("TURN_LEFT", timeToRotate360 / 8, driveSpeed);
        stopRobot();
        while(timer.seconds() <= timeToLiftHopper){
            setArmSpeed("HOPPER", 1);
        }
        timer.reset();
        armBrake("HOPPER");
        moveForTime("BACKWARD", 0.01, driveSpeed);
        timer.reset();
        while(timer.seconds() <= 1) {
            hardware.door.setPosition(open);
        }
        for(int i = 0; i < 4; i++){
            wiggle();
        }
        moveForTime("FORWARD", timeToTravel1Tile * 0.5, driveSpeed);
        stopRobot();
        while(timer.seconds() <= timeToLiftHopper){
            setArmSpeed("HOPPER", -1);
        }
        timer.reset();
        while(timer.seconds() <= 1) {
            hardware.door.setPosition(close);
        }
        timer.reset();
        armBrake("HOPPER");
        moveForTime("TURN_LEFT", (timeToRotate360*2.795)/16, driveSpeed);
    }
}