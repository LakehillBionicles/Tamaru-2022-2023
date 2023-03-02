package org.firstinspires.ftc.teamcode.Tamaru1.Auto1.oldPaths;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Tamaru1.Auto1.AutoBase;

@Autonomous
@Disabled

public class BlueCornerJustParkPID extends AutoBase {


    public double armSpeed = 0.5;

    public void runOpMode(){
        super.runOpMode();
        startUp();
        waitForStart();
        String color = "";

        while (opModeIsActive()){

            robot.BOW.setPower(armSpeed);
            robot.POW.setPower(armSpeed);
            robot.SOW.setPower(armSpeed);
            robot.arm2.setPower(armSpeed);


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


            //move over so in center of tile -- numbers probably wrong
            coordinateDrive(0,-13,0,0.1, 0.1, 0.1, 2, 0.1, 0.1);

            //park based on color sleeve should be good yay!!!!!!!!!
            if(color.equals("red")){
                coordinateDrive(0,0,24,0.1,0.1,0.1,2,0.1, 0.1);

            }else if(color.equals("blue")){
                stop();


            }else{
                coordinateDrive(0,0,-24,0.1,0.1,0.1,2,0.1, 0.1);
            }

            stop();
        }
    }






}
