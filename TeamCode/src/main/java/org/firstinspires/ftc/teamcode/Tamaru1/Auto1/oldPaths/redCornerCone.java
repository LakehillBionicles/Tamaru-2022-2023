package org.firstinspires.ftc.teamcode.Tamaru1.Auto1.oldPaths;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Tamaru1.Auto1.AutoBase;

@Autonomous
@Disabled

public class redCornerCone extends AutoBase {

    ///////////////////////TEST THESE NUMBERS -- I GUESSED ON ALL THESE////////////////////////////////////
    //DELETE THIS COMMENT WHEN THIS AUTO WORKS PRETTY GOOD/////////////////////////////////////////////



    public void runOpMode(){
        super.runOpMode();
        startUp();
        waitForStart();
        String color = "";

        while (opModeIsActive()){



            robot.BOW.setPower(0.5);
            robot.arm2.setPower(0.5);
            sideways(0.5, 16,16, 3); //move to cone

            //senseColorsPort(); //sense colors and store variable for later
            if(senseColorsFront().equals( "red")){
                color = "red";
                telemetry.addData("color", "red");
            }else if(senseColorsFront().equals("blue")){
                telemetry.addData("color", "blue");
                color = "blue";
            }
            telemetry.update();
            sleep(500); //wait to make sure it sees colors
            //turn(0.5, -14, -14, 3);
            sideways(0.4, 25, 25, 3); //c// ont sideways
            turn(0.5, -10, -10, 1);//It turns to currect for sideways not driving straight
            encoderDrive(0.4, -16, -16, 3); //back up to pole
            sleep(100);//port is negative and starboard is positive
            turn(0.5,-10,-10,1);
            sideways(0.4, 10, 10, 3);//sideways so in front of pole

            //armLift(1.0, 900, 4); //arm up -- double check inches here

            //driveUntilDist(0.2,3); couldn't make this work
            encoderDrive(0.4, 1,1,3);
            handDrop(); //open hand
            handDrop();
            sleep(2000);
            handDrop();
            handDrop();
            encoderDrive(0.4, -2, -2, 3);
            sleep(1000);
            //armLift(-1.0, -900, 4); //arm down -- double check inches here
            //sleep(10); //small wait to make sure hand has dropped -- we might be able to delete this part
            turn(0.4, -15,15,3);
            sideways(0.4, -30, -30, 6); //sideways away from pole
            turn(-0.5, 10, 10, 3);

            if(color.equals("red")){
                encoderDrive(0.4,36, 36,6);


            }else if(color.equals("blue")){
                encoderDrive(0.4, 15, 15, 6); //go back to center tiles where we started

            }else{
                encoderDrive(0.4,4, 4, 3);
                stop();
                //already in correct parking spot here??
            }



            //parking method - deleted sideways strafe just b/c I don't think we need it

            stop();
        }
    }



}
