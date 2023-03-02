package org.firstinspires.ftc.teamcode.Tamaru2.Auto2.OldPaths;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Tamaru2.BaseClasses.AutoBase2;

@Autonomous
@Disabled

public class justParkBlueCorner2 extends AutoBase2 {

    public void runOpMode() {
        super.runOpMode();
        startUp();
        waitForStart();
        String color = "";

        while (opModeIsActive()) {
            //coordinateDrive(3, -19, 0, .1, .1, .1, 2, .5, .5);
            //strafe sideways to see cone

            if(senseColorsPort().equals( "green")){
                color = "green";
                telemetry.addData("color", "green");
            }else if(senseColorsPort().equals("blue")){
                telemetry.addData("color", "blue");
                color = "blue";
            }
            telemetry.update();


            sleep(200);

            //coordinateDrive(0, -8, 0, .2, .2, .2, 4, .25, .25);

            if(color.equals("green")){
                //coordinateDrive(0, 0, 23, .2, .2, .2, 4, .25, .25);


            }else if(color.equals("blue")){
                //coordinateDrive(0, -22, 0, .2, .2, .2, 4, .25, .25);

            }else{
                //coordinateDrive(0, 0, -24, .2, .2, .2, 4, .25, .25);
            }

            stop();

        }
    }
}