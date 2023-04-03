package org.firstinspires.ftc.teamcode.Tamaru1.TeleOp1;


import static org.firstinspires.ftc.teamcode.Tamaru1.TemaruHardware.armSpeed;
import static org.firstinspires.ftc.teamcode.Tamaru1.TemaruHardware.closeHandPos;
import static org.firstinspires.ftc.teamcode.Tamaru1.TemaruHardware.openHandPos;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Tamaru1.TemaruHardware;
import org.firstinspires.ftc.teamcode.Tamaru1.Auto1.AutoBase;

@TeleOp
@Disabled
//@Disabled
//VICTORIA DID THIS
//ANNA DID THIS

//////////////////////gamepad1 is drive; gamepad 2 is arm/hand/pre-set distances//////////////////////


public class TeleOp2Drivers extends LinearOpMode { //gamepad1 is drive; gamepad 2 is arm/hand/pre-set distances
    TemaruHardware robot = new TemaruHardware();


    public static double testDriveSpeed = 1.0;
    double speed = 1;
    double speedStrafe = 0.7;
    double startHeading;
    boolean isMoving;

    ////////////////////////////////////////////////////  ELBOW VARIABLES ///////////////////////////////////////////////
    public double elbowTheta;
    public double elbowTargetTheta;
    public double elbowDenominator;
    public double elbowPosition;
    public double elbowError;
    public double lastElbowError;
    public double elbowTime = 0;
    public int    newElbowTarget;
    public double elbowDeriv;
    public double lastElbowDeriv;
    public double elbowIntegralSum = 0;
    public double elbowIntegralSumLimit = 0.25;
    public double elbowTol;
    public double elbowMotorPower;

    public double elbowKp = 1.0;
    public double elbowKd = 1.0;
    public double elbowKi = 1.0;

    public double elbowPower;


    public double currentTheta;
    public double thetaPower;
    public double currentElbowPos;



    public ElapsedTime runtime = new ElapsedTime(); //might cause an error (not sure)


    public void runOpMode() {

        robot.init(hardwareMap);

        robot.arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.arm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();

        telemetry.addData("does this work", "yes it does");
        telemetry.addData("update?", "yes 4");
        telemetry.update();


        while (opModeIsActive()) {

            if(gamepad1.left_stick_x > 0.2 || gamepad1.left_stick_x < -0.2 || gamepad1.left_stick_y > 0.2 ||gamepad1.left_stick_y < -0.2){
                //if you move the left joystick, run the DRIVE method until the joystick goes back to "zero"

                while(gamepad1.left_stick_x > 0.2 || gamepad1.left_stick_x < -0.2 || gamepad1.left_stick_y > 0.2 ||gamepad1.left_stick_y < -0.2) {

                    drive();
                }
                robot.fpd.setPower(0);
                robot.bpd.setPower(0);
                robot.fsd.setPower(0);
                robot.bsd.setPower(0);
            }else if(gamepad1.right_stick_x > 0.15 || gamepad1.right_stick_x < -0.15&& gamepad1.left_stick_y==0 &&  gamepad1.left_stick_x==0){
                //if you move the right joystick, run the TURN method until the joystick goes back to "zero"
                while(gamepad1.right_stick_x > 0.15 || gamepad1.right_stick_x < -0.15&& gamepad1.left_stick_y==0 &&  gamepad1.left_stick_x==0){
                    turn();
                }
                robot.fpd.setPower(0);
                robot.bpd.setPower(0);
                robot.fsd.setPower(0);
                robot.bsd.setPower(0);
            }else{
                robot.fpd.setPower(0);
                robot.bpd.setPower(0);
                robot.fsd.setPower(0);
                robot.bsd.setPower(0);
            }

            while(gamepad2.dpad_up) {
                robot.arm2.setPower(1);
                telemetry.addData("going ", "back");
                telemetry.addData("power", robot.arm2.getPower());
                telemetry.update();
            }

            while(gamepad2.dpad_down) {
                robot.arm2.setPower(-1);
                telemetry.addData("going ", "forward");
                telemetry.addData("power", robot.arm2.getPower());
                telemetry.update();
            }


            //NEED
            //if you press a bumper, run the ARMLIFT method until the bumper is "unpressed"
            armLift();
            //servoHand();
            //servoHandZeroInput();
            setArmToHeight();
            displayDistance();
            //fourBarThetaGoBack();
            //fourBarThetaGoForward();
            servoHandLastTry();
            //fourBarPID2();


        }
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


    public void turn() {
        if (gamepad1.right_stick_x > 0.15 || gamepad1.right_stick_x < -0.15 && gamepad1.left_stick_y==0 &&  gamepad1.left_stick_x==0) {    //clockwise
            //isMoving = true; Don't know what this does might need it
            robot.fpd.setPower(-gamepad1.right_stick_x * speed);
            robot.bpd.setPower(gamepad1.right_stick_x * speed);
            robot.fsd.setPower(gamepad1.right_stick_x * speed);
            robot.bsd.setPower(-gamepad1.right_stick_x * speed);
    /*}else if((gamepad1.right_stick_x > -1 && gamepad1.right_stick_x < -0.2) && (gamepad1.right_stick_y < -.25 && gamepad1.right_stick_y > -1)){    //front port (left)
            robot.fpd.setPower(0.0);
            robot.bpd.setPower(gamepad1.right_stick_y);
            robot.bsd.setPower(0.0);
            robot.fsd.setPower(gamepad1.right_stick_y);
        telemetry.addData("fp","");
        }else if(gamepad1.right_stick_x > 0.2 && gamepad1.right_stick_x < 1 && gamepad1.right_stick_y > -1 && gamepad1.right_stick_y < -0.25){  //front star (right)
            robot.fpd.setPower(gamepad1.right_stick_y);
            robot.bpd.setPower(0.0);
            robot.bsd.setPower(gamepad1.right_stick_y);
            robot.fsd.setPower(0.0);
        telemetry.addData("fs","");
        }else if(gamepad1.right_stick_x > -1 && gamepad1.right_stick_x < -0.25  && gamepad1.right_stick_y > 0.25 && gamepad1.right_stick_y < 1){  //back port
            robot.fpd.setPower(gamepad1.right_stick_y);
            robot.bpd.setPower(0.0);
            robot.bsd.setPower(gamepad1.right_stick_y);
            robot.fsd.setPower(0.0);
        telemetry.addData("bp","");
        }else if(gamepad1.right_stick_x > 0.2 && gamepad1.right_stick_x < 1 && gamepad1.right_stick_y < 1 && gamepad1.right_stick_y > 0.25){   //back star
            robot.fpd.setPower(0.0);
            robot.bpd.setPower(gamepad1.right_stick_y);
            robot.bsd.setPower(0.0);
            robot.fsd.setPower(gamepad1.right_stick_y);
            telemetry.addData("bs","");
*/
        } else {
            robot.fpd.setPower(0);
            robot.bpd.setPower(0);
            robot.fsd.setPower(0);
            robot.bsd.setPower(0);

        }
    }

    public void drive() {


        if (gamepad1.left_stick_x > 0.2 && gamepad1.right_stick_y < 0.2 && gamepad1.right_stick_y > -0.2) {//right

            robot.bpd.setPower((- (speedStrafe * (gamepad1.left_stick_x+(gamepad1.right_stick_x * .8)))));
            robot.fpd.setPower(- 2*(speedStrafe * (gamepad1.left_stick_x-(gamepad1.right_stick_x * .8))));
            robot.bsd.setPower((speedStrafe * (gamepad1.left_stick_x-(gamepad1.right_stick_x * .8))));
            robot.fsd.setPower(2*(speedStrafe * (gamepad1.left_stick_x+(gamepad1.right_stick_x * .8))));
        } else if (gamepad1.left_stick_x < -0.2 && gamepad1.right_stick_y < 0.2 && gamepad1.right_stick_y > -0.2) {//left

            robot.bpd.setPower((- (speedStrafe * (gamepad1.left_stick_x+(gamepad1.right_stick_x * .8)))));
            robot.fpd.setPower(- 2*(speedStrafe * (gamepad1.left_stick_x-(gamepad1.right_stick_x * .8))));
            robot.bsd.setPower((speedStrafe * (gamepad1.left_stick_x-(gamepad1.right_stick_x * .8))));
            robot.fsd.setPower(2*(speedStrafe * (gamepad1.left_stick_x+(gamepad1.right_stick_x * .8))));
        } else if (gamepad1.left_stick_y > 0.2) {//forward
            robot.fpd.setPower(gamepad1.left_stick_y-(gamepad1.right_stick_x * .8));// && (gamepad1.right_stick_x < 0.2 && gamepad1.right_stick_x > -0.2)
            robot.bsd.setPower(-gamepad1.left_stick_y-(gamepad1.right_stick_x * .8));
            robot.fsd.setPower(gamepad1.left_stick_y+(gamepad1.right_stick_x * .8));
            robot.bpd.setPower(-gamepad1.left_stick_y+(gamepad1.right_stick_x * .8));
        } else if (gamepad1.left_stick_y < -0.2){//backward
            robot.fpd.setPower(gamepad1.left_stick_y-(gamepad1.right_stick_x * .8));
            robot.bsd.setPower(-gamepad1.left_stick_y- (gamepad1.right_stick_x * .8));//(gamepad1.right_stick_x < 0.2 && gamepad1.right_stick_x > -0.2)
            robot.fsd.setPower(gamepad1.left_stick_y+(gamepad1.right_stick_x * .8));
            robot.bpd.setPower(-gamepad1.left_stick_y+(gamepad1.right_stick_x * .8));
        } else {
            robot.fpd.setPower(0);
            robot.bpd.setPower(0);
            robot.fsd.setPower(0);
            robot.bsd.setPower(0);
        }
    }


    public void armLift(){
        if (gamepad2.left_stick_y < -0.2) {   //used bumpers here previously//
            robot.BOW.setPower(1);
            robot.SOW.setPower(-1);
        } else if (gamepad2.right_stick_y > 0.2){
            robot.BOW.setPower(-1);
            robot.SOW.setPower(1);
        } else if (!(gamepad2.left_stick_y < -0.2) && !(gamepad2.right_stick_y > 0.2) && !gamepad2.a && !gamepad2.b && !gamepad2.y && !gamepad2.x){
            robot.BOW.setPower(0.0);
            robot.SOW.setPower(0.0);
        } else {}
    }

    public void setArmToHeight(){
        if (gamepad2.a) { //cone distance
            robot.BOW.setPower(2 * -(Math.sin((robot.distSensorArm.getDistance(DistanceUnit.CM) + robot.distSensorHighArm.getDistance(DistanceUnit.CM) + robot.distSensorLowerArm.getDistance(DistanceUnit.CM) - 25) * (3.1415 / 4) / 85)));
            robot.SOW.setPower(-1 * (2 * -(Math.sin((robot.distSensorArm.getDistance(DistanceUnit.CM) + robot.distSensorHighArm.getDistance(DistanceUnit.CM) + robot.distSensorLowerArm.getDistance(DistanceUnit.CM)- 25) * (3.1415 / 4) / 85))));
        }
        if (gamepad2.x){ //small distance
            robot.BOW.setPower(2 * -(Math.sin((robot.distSensorArm.getDistance(DistanceUnit.CM) + robot.distSensorHighArm.getDistance(DistanceUnit.CM) + robot.distSensorLowerArm.getDistance(DistanceUnit.CM)  - 60) * (3.1415 / 4) / 85)));
            robot.SOW.setPower(-1 * (2 * -(Math.sin((robot.distSensorArm.getDistance(DistanceUnit.CM) + robot.distSensorHighArm.getDistance(DistanceUnit.CM) + robot.distSensorLowerArm.getDistance(DistanceUnit.CM) - 60) * (3.1415 / 4) / 85))));
        }
        if (gamepad2.y){ //medium distance
            robot.BOW.setPower(2 * -(Math.sin((robot.distSensorArm.getDistance(DistanceUnit.CM) + robot.distSensorHighArm.getDistance(DistanceUnit.CM) + robot.distSensorLowerArm.getDistance(DistanceUnit.CM)- 94) * (3.1415 / 4) / 85)));
            robot.SOW.setPower(-1 * (2 * -(Math.sin((robot.distSensorArm.getDistance(DistanceUnit.CM) + robot.distSensorHighArm.getDistance(DistanceUnit.CM) + robot.distSensorLowerArm.getDistance(DistanceUnit.CM)- 94) * (3.1415 / 4) / 85))));
        }
        if (gamepad2.b){ //large distance -- might need to comment out
            robot.BOW.setPower(2 * -(Math.sin((robot.distSensorArm.getDistance(DistanceUnit.CM) + robot.distSensorHighArm.getDistance(DistanceUnit.CM) + robot.distSensorLowerArm.getDistance(DistanceUnit.CM)- 98) * (3.1415 / 4) / 85)));
            robot.SOW.setPower(-1 * (2 * -(Math.sin((robot.distSensorArm.getDistance(DistanceUnit.CM) + robot.distSensorHighArm.getDistance(DistanceUnit.CM) + robot.distSensorLowerArm.getDistance(DistanceUnit.CM)- 98) * (3.1415 / 4) / 85))));
        }


        /* MATH EXPLAINED BELOW /////////////////////////////////////////////////////////////////////////////////////////

         * 2 * --> adds more power to the arm
         * - sign --> fixes direction
         * sin (((current distances of all sensors added together - target distance) * pi/4) /total height of arm)
         *
         * *////////////////////////////////////////////////////////////////////////////////////////////////////////


        //(sin(|current - target|) / max height ) * pi / 4

        //target pos - current pos = how far away
        //set speed based on
        //power is function of __ or power is minus function of ___
        //absolute value around difference

        //if current - target is here this power
        // if current - target is here this power


    }


    public void fourBarThetaGoBack() {

        while(gamepad2.dpad_up) {
            robot.arm2.setPower(1);
            telemetry.addData("going ", "back");
            telemetry.addData("power", robot.arm2.getPower());
            telemetry.update();
        }

        /*if (gamepad2.dpad_up) {
            newElbowTarget = 180;

            robot.arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            elbowPosition = robot.arm2.getCurrentPosition();
            elbowTol = .05;
            elbowTargetTheta = ((Math.PI / 144) * newElbowTarget);
            elbowTheta = ((Math.PI / 144) * elbowPosition);
            elbowError = Math.abs(elbowTargetTheta - elbowTheta);

            telemetry.addData("in loop", "yes");
            telemetry.update();

            while (Math.abs(elbowError) > elbowTol) {

                /*if ((elbowError) >= (3.2)) {
                    elbowPower = (1);
                        //start moving
                } else if ((elbowError) >= (1.6) && (elbowError) < (3.2)) {
                    elbowPower = .75 * (Math.abs(Math.cos(elbowTheta - 1)));
                        //keep turning but not as fast
                } else if (((elbowError) < (1.2)) && (elbowError > 1.6)) {
                    elbowPower = .8 * (Math.cos(elbowTheta)); //we tuned the number that multiplies the cos function
                        //breaking
                } else {
                    elbowPower = 0;
                    //resting at target and just before breaking
                }

                robot.arm2.setPower(elbowPower);

                elbowPosition = robot.arm2.getCurrentPosition();
                elbowTheta = ((Math.PI / 144) * elbowPosition);
                elbowError = Math.abs(elbowTargetTheta - elbowTheta);
                elbowMotorPower = robot.arm2.getPower();


                telemetry.addData("theoretical power", elbowPower);
                telemetry.addData("actual power", elbowMotorPower);
                telemetry.addData("error", elbowError);
                telemetry.update(); */

    }

    public void fourBarThetaGoForward() {

        while (gamepad2.dpad_down) {
            robot.arm2.setPower(-1);
            telemetry.addData("going ", "forward");
            telemetry.addData("power", robot.arm2.getPower());
            telemetry.update();
        }
           /*
        if (gamepad2.dpad_down) {
            newElbowTarget = 0;

            robot.arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            elbowPosition = robot.arm2.getCurrentPosition();
            elbowTol = .05;
            elbowTargetTheta = ((Math.PI / 144) * newElbowTarget);
            elbowTheta = ((Math.PI / 144) * elbowPosition);
            elbowError = Math.abs(elbowTargetTheta - elbowTheta);

            telemetry.addData("in loop", "yes");
            telemetry.update();

            while (Math.abs(elbowError) > elbowTol) {

                if ((elbowError) >= (2.3)) {
                    elbowPower = (-1);

                } else if ((elbowError) >= (.5) && (elbowError) < (2.3)) {
                    elbowPower = .75 * (Math.abs(Math.cos(elbowTheta - 1)));

                } else if (((elbowError) < (.5)) && (elbowError > elbowTol)) {
                    elbowPower = .5 * (Math.cos(elbowTheta));

                } else {
                    elbowPower = 0;
                }

                robot.arm2.setPower(elbowPower);

                elbowPosition = robot.arm2.getCurrentPosition();
                elbowTheta = ((Math.PI / 144) * elbowPosition);
                elbowError = Math.abs(elbowTargetTheta - elbowTheta);
                elbowMotorPower = robot.arm2.getPower();


                telemetry.addData("theoretical power", elbowPower);
                telemetry.addData("actual power", elbowMotorPower);
                telemetry.addData("error", elbowError);
                telemetry.update();

            }

        }

            */
    }

    public void servoHandLastTry() {
        if (gamepad2.left_trigger > 0) {
            telemetry.addData("please work", "yay");
            telemetry.update();
            robot.servoFinger.setPosition(1.0);

        } else {
            robot.servoFinger.setPosition(0.0);
            //robot.servoFinger.setPosition(.7);
        }
    }



    /*public void servoHand(){
        if (gamepad2.right_trigger > 0){
            telemetry.addData("hand pos", "open");
            telemetry.update();
            robot.servoFinger.setPosition(1.0);

        } else if (gamepad2.left_trigger > 0){
            telemetry.addData("hand pos:", "closed");
            telemetry.update();
            robot.servoFinger.setPosition(0.45);
        } else {
        }
    }

    public void servoHandZeroInput(){
        if ((robot.distSensorHand.getDistance(DistanceUnit.CM) < 3) && !(gamepad2.right_trigger > 0)){
            telemetry.addData("i see:", "cone");
            telemetry.update();
            robot.servoFinger.setPosition(0.45);
        }


    }*/




    public void displayDistance(){
        telemetry.addData("upper arm:", robot.distSensorArm.getDistance(DistanceUnit.CM));
        telemetry.addData("lower arm:", robot.distSensorLowerArm.getDistance(DistanceUnit.CM));
        telemetry.addData("middle arm:", robot.distSensorHighArm.getDistance(DistanceUnit.CM));
        telemetry.addData("distance:", robot.distSensorArm.getDistance(DistanceUnit.CM) + robot.distSensorLowerArm.getDistance(DistanceUnit.CM) + robot.distSensorHighArm.getDistance(DistanceUnit.CM));

//commented out telemtry for four bar to make better

        telemetry.addData("hand", robot.distSensorHand.getDistance(DistanceUnit.CM));
        telemetry.update();



        //small distance 39
        //medium distance 47
        //big distance
        // pick up cone from ground dist 15



    }




    public void doLights(){
        if (robot.touchSensorPort.isPressed() && robot.touchSensorStar.isPressed()){
            robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_WITH_GLITTER);
            //} else if (robot.distSensorHand.getDistance(DistanceUnit.CM) < 8){
            // robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
        } else if (robot.touchSensorArm.isPressed()){
            robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED); //this should make it works with the dist. sensor
        }else {
            robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        }
    }




}
