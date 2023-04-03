package org.firstinspires.ftc.teamcode.Tamaru1.Auto1.currentPaths;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Tamaru1.Auto1.AutoBase;

@Autonomous
@Disabled

public class JustParkRedCorner extends AutoBase {

    public void runOpMode() {
        super.runOpMode();
        startUp();
        waitForStart();
        String color = "";


        while (opModeIsActive()) {
            //I lowered rotTol from 4 to 2 which made lining up with the cone much easier and only cost 1 second
            coordinateDrive(0, 19, 0, .1, .1, .1, 2, .25, .25);
            //strafe sideways to see cone

            if(senseColorsFront().equals( "red")){
                color = "red";
                telemetry.addData("color", "red");
            }else if(senseColorsFront().equals("blue")){
                telemetry.addData("color", "blue");
                color = "blue";
            }
            telemetry.update();


            sleep(200);

            coordinateDrive(0, 8, 0, .2, .2, .2, 4, .25, .25);

            if(color.equals("red")){ //I added a 2 inch strafe because we were going over the line
                coordinateDrive(0, 0, 26, .2, .2, .2, 4, .25, .25);

            }else if(color.equals("blue")){
                coordinateDrive(0, 22, 0, .2, .2, .2, 4, .25, .25);

            }else{ //changed yTarget from -24 to -23 because we were getting a little close to the edge
                coordinateDrive(0, 0, -23, .2, .2, .2, 4, .25, .25);
            }




            stop();

        }
    }
}