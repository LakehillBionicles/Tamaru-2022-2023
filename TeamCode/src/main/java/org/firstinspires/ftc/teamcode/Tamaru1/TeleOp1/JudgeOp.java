package org.firstinspires.ftc.teamcode.Tamaru1.TeleOp1;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Tamaru1.TemaruHardware;
import org.firstinspires.ftc.teamcode.Tamaru1.Auto1.AutoBase;

@TeleOp
@Disabled

public class JudgeOp extends LinearOpMode {


    TemaruHardware robot = new TemaruHardware();


    public static double testDriveSpeed = 1.0;
    double speed = 1;
    double speedStrafe = 0.7;
    double startHeading;
    boolean isMoving;

    public void runOpMode() {

        robot.init(hardwareMap);

        waitForStart();

        telemetry.addData("does this work", "yes it does");
        telemetry.update();


        while (opModeIsActive()) {



            //NEED
            //if you press a bumper, run the ARMLIFT method until the bumper is "unpressed"
            armLift();
            doLights();
            //hand();
            servoHand();
            setArmToHeight();
            displayDistance();

        }

       /* public void drive(double stickX double stickY) { //omni strafe ||Testing||
            double angle = Math.atan2(stickY, stickX);
            double magnitude = Math.sqrt(Math.pow(stickY, 2) + Math.pow(stickX, 2));
            if (stickX > 0.2 || stickX < -0.2 || stickY < -0.2 || stickY > 0.2) {
                isMoving = true;
                fsd.setPower((Math.sin(angle + Math.PI / 4)) * magnitude * speed);//cos maybe?
               bpd.setPower((Math.sin(angle + Math.PI / 4)) * magnitude * speed);
               fpd.setPower((Math.sin(angle - Math.PI / 4)) * magnitude * speed);
                bsd.setPower((Math.sin(angle - Math.PI / 4)) * magnitude * speed);
            }*/
    }


    public void armLift(){

        //sldkfjalsdhgsadhf;ajsdlfj

        if (gamepad1.left_bumper) { //goes up
            robot.BOW.setPower(1);
            robot.arm2.setPower(1);
        } else if (gamepad1.right_bumper){
            robot.BOW.setPower(-0.4);
            robot.arm2.setPower(-0.4);
        } else if (!(gamepad1.left_bumper) && !(gamepad1.right_bumper) && !gamepad1.a && !gamepad1.b && !gamepad1.y && !gamepad1.x){
            robot.BOW.setPower(0.1);
            robot.arm2.setPower(0.1);
        } else {}
    }

    /*public void hand(){
        if (gamepad1.left_trigger > 0){
            robot.hand.setPower(1.0);
        } else if (gamepad1.right_trigger > 0){
            robot.hand.setPower(-1.0);
        } else {
            robot.hand.setPower(0.0);
        }
        //for reference w/in the setPos we had this instead: (robot.flippyBox.getPosition() + .081)

    }*/

    public void servoHand(){
        if (gamepad1.left_trigger > 0){
            robot.servoFinger.setPosition(0.0);
        } else {
            robot.servoFinger.setPosition(1.0);
        }
        //for reference w/in the setPos we had this instead: (robot.flippyBox.getPosition() + .081)

    }

    public void displayDistance(){
        telemetry.addData("upper arm:", robot.distSensorArm.getDistance(DistanceUnit.CM));
        telemetry.addData("lower arm:", robot.distSensorLowerArm.getDistance(DistanceUnit.CM));
        telemetry.addData("middle arm:", robot.distSensorMiddleArm.getDistance(DistanceUnit.CM));
        telemetry.addData("distance:", robot.distSensorArm.getDistance(DistanceUnit.CM) + robot.distSensorLowerArm.getDistance(DistanceUnit.CM) + robot.distSensorMiddleArm.getDistance(DistanceUnit.CM));

        telemetry.addData("horizontal", robot.distSensorHand.getDistance(DistanceUnit.CM));
        telemetry.update();

        //small distance 39
        //medium distance 47
        //big distance
        // pick up cone from ground dist 15



    }

    public void setArmToHeight(){
        if (gamepad1.a) { //cone distance
            robot.BOW.setPower(2 * -(Math.sin((robot.distSensorArm.getDistance(DistanceUnit.CM) + robot.distSensorMiddleArm.getDistance(DistanceUnit.CM) + robot.distSensorLowerArm.getDistance(DistanceUnit.CM) - 25) * (3.1415 / 4) / 85)));
            robot.arm2.setPower(2 * -(Math.sin((robot.distSensorArm.getDistance(DistanceUnit.CM) + robot.distSensorMiddleArm.getDistance(DistanceUnit.CM) + robot.distSensorLowerArm.getDistance(DistanceUnit.CM)- 25) * (3.1415 / 4) / 85)));
        }
        if (gamepad1.x){ //small distance

            robot.BOW.setPower(2 * -(Math.sin((robot.distSensorArm.getDistance(DistanceUnit.CM) + robot.distSensorMiddleArm.getDistance(DistanceUnit.CM) + robot.distSensorLowerArm.getDistance(DistanceUnit.CM)  - 60) * (3.1415 / 4) / 85)));
            robot.arm2.setPower(2 * -(Math.sin((robot.distSensorArm.getDistance(DistanceUnit.CM) + robot.distSensorMiddleArm.getDistance(DistanceUnit.CM) + robot.distSensorLowerArm.getDistance(DistanceUnit.CM) - 60) * (3.1415 / 4) / 85)));
        }

        if (gamepad1.y){ //medium distance

            robot.BOW.setPower(2 * -(Math.sin((robot.distSensorArm.getDistance(DistanceUnit.CM) + robot.distSensorMiddleArm.getDistance(DistanceUnit.CM) + robot.distSensorLowerArm.getDistance(DistanceUnit.CM)- 94) * (3.1415 / 4) / 85)));
            robot.arm2.setPower(2 * -(Math.sin((robot.distSensorArm.getDistance(DistanceUnit.CM) + robot.distSensorMiddleArm.getDistance(DistanceUnit.CM) + robot.distSensorLowerArm.getDistance(DistanceUnit.CM)- 94) * (3.1415 / 4) / 85)));
        }


        if (gamepad1.b){ //large distance -- might need to comment out

            robot.BOW.setPower(2 * -(Math.sin((robot.distSensorArm.getDistance(DistanceUnit.CM) + robot.distSensorMiddleArm.getDistance(DistanceUnit.CM) + robot.distSensorLowerArm.getDistance(DistanceUnit.CM)- 98) * (3.1415 / 4) / 85)));
            robot.arm2.setPower(2 * -(Math.sin((robot.distSensorArm.getDistance(DistanceUnit.CM) + robot.distSensorMiddleArm.getDistance(DistanceUnit.CM) + robot.distSensorLowerArm.getDistance(DistanceUnit.CM)- 98) * (3.1415 / 4) / 85)));
        }


        /* MATH EXPLAINED BELOW /////////////////////////////////////////////////////////////////////////////////////////

         * 2 * --> adds more power to the arm
         * - sign --> fixes direction
         * sin (((current distances of all sensors added together - target distance) * pi/4) /total height of arm)
         *
         * *////////////////////////////////////////////////////////////////////////////////////////////////////////

           /* } else if (robot.distSensorArm.getDistance(DistanceUnit.CM) < 13){
                robot.BOW.setPower(1.0);
                robot.arm2.setPower(1.0);

            } else {
              robot.BOW.setPower(0.0);
              robot.arm2.setPower(0.0);*/

        //(sin(|current - targer|) / max height ) * pi / 4

        //target pos - current pos = how far away
        //set speed based on
        //power is function of __ or power is minus function of ___
        //absolute value around difference

        //if current - target is here this power
        // if current - target is here this power


    }







    public void doLights(){
        if (robot.touchSensorPort.isPressed() && robot.touchSensorStar.isPressed()){
            robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_WITH_GLITTER);
        } else if (robot.distSensorHand.getDistance(DistanceUnit.CM) < 8){
            robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
        } else if (robot.touchSensorArm.isPressed()){
            robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED); //this should make it works with the dist. sensor
        }else {
            robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        }
    }

    /*public void magnetMagic(){
        if (robot.magnet.isPressed()){ //is pressed at 1 cm
            telemetry.addData("it works", "yay");
            telemetry.update();
        } else {
            telemetry.addData("no work", "sad");
            telemetry.update();

        }


    }*/



        /* public void testDrive(){





    }*/

}
