package org.firstinspires.ftc.teamcode.Tamaru1.Auto1.oldPaths;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Tamaru1.Auto1.AutoBase;

@Autonomous
@Disabled
////////////////////////CHECK ALL THESE NUMBERS -- I JUST GUESSED ON THEM////////////////////////////
///////////////////DELETE THIS COMMENT WHEN AUTO WORK OKAYISH////////////////////////////////////////////

public class RedSideBlueTerminalCone extends AutoBase {

    public void runOpMode(){
        super.runOpMode();
        startUp();
        waitForStart();
        String color = "";

        while (opModeIsActive()){


            //turn(0.2, 90, 90, 10);

            //both hand and arm functions should now work -- just focus on gettng the paths down today



            sideways(0.5, -16,-16, 3); //move to cone

            //senseColorsPort(); //sense colors and store variable for later
            if(senseColorsPort().equals( "red")){
                color = "red";
                telemetry.addData("color", "red");
            }else if(senseColorsPort().equals("blue")){
                telemetry.addData("color", "blue");
                color = "blue";
            }
            telemetry.update();
            sleep(500); //wait to make sure it sees colors
            sideways(0.4, -30, -30, 3); //c// ont sideways
            turn(0.5, 28, 28, 1);//It turns to currect for sideways not driving straight
            encoderDrive(0.4, -11, -11, 3); //back up to pole
            sleep(100);//port is negative and starboard is positive
            turn(0.5,10,10,1);
            sideways(0.4, -11.5, -11.5, 3);//sideways so in front of pole
            armLift(1.0, 900, 4); //arm up -- double check inches here
            encoderDrive(0.4, 3, 3, 4);
            //driveUntilDist(0.2,3); couldn't make this work
            handDrop(); //open hand
            handDrop();
            armLift(1.0, 50,2);
            encoderDrive(0.4, -2, -2, 3);
            armLift(-1.0, -900, 4); //arm down -- double check inches here
            sleep(10); //small wait to make sure hand has dropped -- we might be able to delete this part
            turn(0.4, -15, -15, 4);
            sideways(0.4, 34, 34, 3); //sideways away from pole

            if(color.equals("red")){
                stop();


            }else if(color.equals("blue")){
                encoderDrive(0.4, 14, 12.5, 3); //go back to center tiles where we started

            }else{
                encoderDrive(0.4,38, 38,3);
                //already in correct parking spot here??
            }



            //parking method - deleted sideways strafe just b/c I don't think we need it

            stop();
        }
    }










}
