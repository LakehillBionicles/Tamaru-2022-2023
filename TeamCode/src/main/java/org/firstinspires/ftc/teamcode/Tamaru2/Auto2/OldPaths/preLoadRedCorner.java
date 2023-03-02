package org.firstinspires.ftc.teamcode.Tamaru2.Auto2.OldPaths;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Tamaru2.BaseClasses.executionClass;

@Autonomous
//@Disabled

public class preLoadRedCorner extends executionClass {

    @Override
    public void runOpMode() throws InterruptedException{
        startUp();
        waitForStart();
        String color = "";

        while(opModeIsActive()){

            //drive to cone
            odoPowerCalculations(0, 19, 0, .1, .1, .1, 2, .25, .25);
            while(!odoCalculationsFinished) {
                execution(fpdPOWOdoPower, bpdBOWOdoPower, fsdSOWOdoPower, bsdOdoPower, 1, 0, 0, 0, 0, 0, true, "auto");
            }

            //sense color
            if(senseColorsStar().equals( "red")){
                color = "red";
                telemetry.addData("color", "green");
            }else if(senseColorsStar().equals("blue")){
                telemetry.addData("color", "blue");
                color = "blue";
            }
            telemetry.update();

            //TODO: write code for arm up


            //drive to pole, this is also blue parking
            odoPowerCalculations(0, 6, 0, .2, .2, .2, 4, .25, .25);
            while(!odoCalculationsFinished) {
                execution(fpdPOWOdoPower, bpdBOWOdoPower, fsdSOWOdoPower, bsdOdoPower, 1, 0, 0, 0, 0, 0, true, "auto");
            }

            if(color.equals("red")){
                //strafe a bit more
                odoPowerCalculations(0, 6, 0, .2, .2, .2, 4, .25, .25);
                while(!odoCalculationsFinished) {
                    execution(fpdPOWOdoPower, bpdBOWOdoPower, fsdSOWOdoPower, bsdOdoPower, 1, 0, 0, 0, 0, 0, true, "auto");
                }
                //drive forward into parking spot
                odoPowerCalculations(0, 0, 12, .2, .2, .2, 4, .25, .25);
                while(!odoCalculationsFinished) {
                    execution(fpdPOWOdoPower, bpdBOWOdoPower, fsdSOWOdoPower, bsdOdoPower, 1, 0, 0, 0, 0, 0, true, "auto");
                }
            }else if(color.equals("green")){
                //strafe a bit more
                odoPowerCalculations(0, 6, 0, .2, .2, .2, 4, .25, .25);
                while(!odoCalculationsFinished) {
                    execution(fpdPOWOdoPower, bpdBOWOdoPower, fsdSOWOdoPower, bsdOdoPower, 1, 0, 0, 0, 0, 0, true, "auto");
                }
                //drive forward into parking spot
                odoPowerCalculations(0, 0, -12, .2, .2, .2, 4, .25, .25);
                while(!odoCalculationsFinished) {
                    execution(fpdPOWOdoPower, bpdBOWOdoPower, fsdSOWOdoPower, bsdOdoPower, 1, 0, 0, 0, 0, 0, true, "auto");
                }
            }

            stop();
        }
    }
}
