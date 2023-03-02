package org.firstinspires.ftc.teamcode.Tamaru1.Auto1.oldPaths;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Tamaru1.Auto1.AutoBase;

@Autonomous
@Disabled
public class BlueCornerPID extends AutoBase {



    public void runOpMode(){
        super.runOpMode();
        startUp();
        waitForStart();
        String color = "";

        while (opModeIsActive()){

            robot.BOW.setPower(-0.1);
            robot.POW.setPower(0.1);

            //move to align with cone
            coordinateDrive(0, -18, 0, 0.1,0.1,0.1,2,0.1,0.1);

            //color sense and store variable
            if(senseColorsPort().equals( "red")){
                color = "red";
                telemetry.addData("color", "red");
            }else if(senseColorsPort().equals("blue")){
                telemetry.addData("color", "blue");
                color = "blue";
            }
            telemetry.update();

            sleep(500); //wait to make sure it sees colors

            //continue sideways and back up to pole
            //THIS MIGHT NOT WORK AND MIGHT RUN INTO POLE THOUGH IT SHOULD SELF CORRECT - MAYBE???
            coordinateDrive(0, -33, -27, 0.1, 0.1, 0.1, 2, 0.1, 0.1);

            //move so we are right in front of pole
            coordinateDrive(0, -11.5, 5, 0.1, 0.1, 0.1, 2, 0.1, 0.1);

            //set arm power here (not sure to be honest)?????

            handDrop();
            handDrop();
            sleep(100);
            handDrop();
            handDrop();

            //back up from pole and move to the right in order to line up with tiles well
            coordinateDrive(0,29.5,-5,0.1, 0.1, 0.1, 2, 0.1, 0.1);

            //park based on color sleeve should be good yay!!!!!!!!!
            if(color.equals("red")){
                stop();

            }else if(color.equals("blue")){
                coordinateDrive(0,0,-27,0.1,0.1,0.1,2,0.1, 0.1);


            }else{
                coordinateDrive(0,0,-51,0.1,0.1,0.1,2,0.1, 0.1);
            }

            stop();
        }
    }

}
