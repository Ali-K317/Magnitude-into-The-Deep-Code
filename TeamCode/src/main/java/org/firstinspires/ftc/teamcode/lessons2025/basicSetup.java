package org.firstinspires.ftc.teamcode.lessons2025;
/*
Don't worry about memorizing these imports.
The 'LinearOpMode' will be red at first, indicating an error,
hover over it to see "import files", press it and it imports it automatically.
This works for all libraries.
 */
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/*
Public means it can be accessed between different files.
Since the other pre-loaded files need to access it, our
main 'class'-the main part of the code- must be public
 */
public class basicSetup extends LinearOpMode {
    //This is where all your motors and variables will be, more on that later
    @Override
    public void runOpMode() throws InterruptedException {
        //This is where your initializing will be, more on that later

        /*
        You must type 'waitForStart()' so that it knows to start
        this loop once the driver hub is pressed.
        If you don't it will immediate start before initialization,
        breaking it.
        This is such a common issue that the driver hub will display
        "Did you forget to add "waitForStart()"?".
         */
        waitForStart();
        while(opModeIsActive()){
            //This is where all your code requiring inputs will be.
        }
    }
}
