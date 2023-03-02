package org.firstinspires.ftc.teamcode.Tamaru1.TeleOp1;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Tamaru1.TemaruHardware;
import org.firstinspires.ftc.teamcode.Tamaru1.Auto1.AutoBase;

@TeleOp
@Disabled

//////////////////////gamepad1 is drive; gamepad 2 is arm/hand/pre-set distances//////////////////////

public class TeleOp2DriversBetter extends LinearOpMode {
    TemaruHardware robot = new TemaruHardware();

    //the below variables are private to keep everything within this class

    //gamepad 1 (drive) variable
    private double drivePower;
    private double strafePower;
    private double rotatePower;

    //gamepad 2 (arm/elbow/hand) variable
    private double armPower;
    private double elbowPower;
    private double servoArmPower;
    private double handPos = .5;

    private double fpdPower;
    private double bpdPower;
    private double fsdPower;
    private double bsdPower;

    private double teleDenom;

    ///////////SWIVEL PID VARIABLES///////////////
    private double arm2Power;

    private double swivelPower;
    private double swivelLocation;
    private double swivelTarget;
    private double swivelError;
    private double swivelTime;
    private double swivelDenominator;
    private double swivelIntegralSumLimit = .25;
    private double swivelIntegralSum;
    private double swivelDerivative;
    private double swivelLastError;
    private double swivelMaxPower = 0.25;
    private double swivelLastTime;
    private double swivelDeltaTime;

    private double swivelKp = .2;
    private double swivelKi = .7;
    private double swivelKd = .1;
    ///////////////////////////////////////////////


    public ElapsedTime runtime = new ElapsedTime();


    public void runOpMode() {

        robot.init(hardwareMap);

        waitForStart();

        telemetry.addData("update date", "____");
        telemetry.update();

        robot.arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        swivelTarget = 0;


        while (opModeIsActive()) {

            ///////////////////////////////////////////////////////////// GAMEPAD 1 //////////////////////////////////////////////////
            if (gamepad1.left_trigger > 0) {
                drivePower = gamepad1.left_stick_y / 2;
                strafePower = -gamepad1.left_stick_x / 2;
                rotatePower = gamepad1.right_stick_x / 2;
            } else {
                drivePower = gamepad1.left_stick_y;
                strafePower = -gamepad1.left_stick_x;
                rotatePower = -gamepad1.right_stick_x;
            }
            /////////////////////////////////////////////////////////// GAMEPAD 2 ////////////////////////////////////////////////////

            //armPower = -gamepad2.left_stick_y;

            if(gamepad2.left_stick_y < 0 && !(gamepad2.left_trigger > 0) && !(gamepad2.right_trigger >0)){
                armPower = -gamepad2.left_stick_y;
                servoArmPower = 1;
            } else if(gamepad2.left_stick_y > 0 && !(gamepad2.left_trigger > 0) && !(gamepad2.right_trigger >0)){
                armPower = -gamepad2.left_stick_y/4;
                servoArmPower = -1;
            } else{
                armPower = 0;
            }

            if (gamepad2.left_trigger > 0) {
                servoArmPower = 1;
            } else if (gamepad2.right_trigger > 0) {
                servoArmPower = -1;
            } else {
                servoArmPower = 0;
            }

            if (gamepad2.left_bumper) {
                handPos = 0.5;
            } else if (gamepad2.right_bumper) {
                handPos = 1;
            }

            if (!(gamepad2.left_bumper)) {
                if (((robot.colorSensorHand.red() > robot.colorSensorHand.green()) || (robot.colorSensorHand.blue() > robot.colorSensorHand.green()))) {
                    if (robot.distSensorHand.getDistance(DistanceUnit.CM) < 8) {
                        handPos = 0.5;
                    }
                }
            }

            /*if (gamepad2.dpad_up) {
                robot.arm2.setTargetPosition(0);
                robot.arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elbowPower = .25;
            } else if (gamepad2.dpad_right) {
                robot.arm2.setTargetPosition(67);
                robot.arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elbowPower = .25;
            } else if (gamepad2.dpad_down) {
                robot.arm2.setTargetPosition(141);
                robot.arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elbowPower = .25;
            } else if (gamepad2.dpad_left) {
                robot.arm2.setTargetPosition(-50);
                robot.arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elbowPower = .25;
            } else {
                elbowPower = 0;
            }*/

            ///////////////////SWIVEL PID/////////////////////////////
            if (gamepad2.dpad_left) {
                swivelTarget = 67;
            } else if (gamepad2.dpad_down) {
                swivelTarget = 141;
            } else if (gamepad2.dpad_right) {
                swivelTarget = -50;
            } else if (gamepad2.dpad_up) {
                swivelTarget = 0;
            }
            robot.arm2.setPower(0.25);

            swivelLocation = robot.arm2.getCurrentPosition();

            swivelError = (swivelTarget - swivelLocation);
            time = getRuntime();
            swivelDenominator = Math.max(Math.abs(swivelPower), 1);

            swivelDeltaTime = time - swivelLastTime;

            swivelDerivative = ((swivelError - swivelLastError) / time);
            swivelIntegralSum = (swivelIntegralSum + (swivelError * time));
            if (swivelIntegralSum > swivelIntegralSumLimit) {
                swivelIntegralSum = swivelIntegralSumLimit;
            }
            if (swivelIntegralSum < -swivelIntegralSumLimit) {
                swivelIntegralSum = -swivelIntegralSumLimit;
            }
            swivelPower = -((swivelKd * swivelDerivative) + (swivelKi * swivelIntegralSum) + (swivelKp * Math.signum(swivelError)));

            swivelLastError = swivelError;
            swivelLastTime = time;
            arm2Power = ((swivelPower * swivelMaxPower) / swivelDenominator);

            /////////////////////////////////////////////////////////////


            if (gamepad2.a) { //cone distance
                robot.BOW.setPower(2 * -(Math.sin((robot.distSensorArm.getDistance(DistanceUnit.CM) + robot.distSensorHighArm.getDistance(DistanceUnit.CM) + robot.distSensorLowerArm.getDistance(DistanceUnit.CM) - 25) * (3.1415 / 4) / 85)));
                robot.SOW.setPower(-1 * (2 * -(Math.sin((robot.distSensorArm.getDistance(DistanceUnit.CM) + robot.distSensorHighArm.getDistance(DistanceUnit.CM) + robot.distSensorLowerArm.getDistance(DistanceUnit.CM) - 25) * (3.1415 / 4) / 85))));
            }
            if (gamepad2.x) { //small distance
                robot.BOW.setPower(2 * -(Math.sin((robot.distSensorArm.getDistance(DistanceUnit.CM) + robot.distSensorHighArm.getDistance(DistanceUnit.CM) + robot.distSensorLowerArm.getDistance(DistanceUnit.CM) - 60) * (3.1415 / 4) / 85)));
                robot.SOW.setPower(-1 * (2 * -(Math.sin((robot.distSensorArm.getDistance(DistanceUnit.CM) + robot.distSensorHighArm.getDistance(DistanceUnit.CM) + robot.distSensorLowerArm.getDistance(DistanceUnit.CM) - 60) * (3.1415 / 4) / 85))));
            }
            if (gamepad2.y) { //medium distance
                robot.BOW.setPower(2 * -(Math.sin((robot.distSensorArm.getDistance(DistanceUnit.CM) + robot.distSensorHighArm.getDistance(DistanceUnit.CM) + robot.distSensorLowerArm.getDistance(DistanceUnit.CM) - 94) * (3.1415 / 4) / 85)));
                robot.SOW.setPower(-1 * (2 * -(Math.sin((robot.distSensorArm.getDistance(DistanceUnit.CM) + robot.distSensorHighArm.getDistance(DistanceUnit.CM) + robot.distSensorLowerArm.getDistance(DistanceUnit.CM) - 94) * (3.1415 / 4) / 85))));
            }
            if (gamepad2.b) { //large distance -- might need to comment out
                robot.BOW.setPower(2 * -(Math.sin((robot.distSensorArm.getDistance(DistanceUnit.CM) + robot.distSensorHighArm.getDistance(DistanceUnit.CM) + robot.distSensorLowerArm.getDistance(DistanceUnit.CM) - 98) * (3.1415 / 4) / 85)));
                robot.SOW.setPower(-1 * (2 * -(Math.sin((robot.distSensorArm.getDistance(DistanceUnit.CM) + robot.distSensorHighArm.getDistance(DistanceUnit.CM) + robot.distSensorLowerArm.getDistance(DistanceUnit.CM) - 98) * (3.1415 / 4) / 85))));
            }

            ////////////////////////////////////////////////////////// MATH //////////////////////////////////////////////////////////
            fpdPower = drivePower + strafePower + rotatePower;
            bpdPower = drivePower - strafePower + rotatePower;
            fsdPower = drivePower - strafePower - rotatePower;
            bsdPower = drivePower + strafePower - rotatePower;

            teleDenom = Math.max(Math.max(Math.abs(fpdPower), Math.abs(bpdPower)), Math.max(Math.abs(fsdPower), Math.abs(bsdPower)));

            ///////////////////////////////////////////////////////// MOTOR POWERS ////////////////////////////////////////////////////
            robot.fpd.setPower(fpdPower / teleDenom);
            robot.fsd.setPower(fsdPower / teleDenom);
            robot.bpd.setPower(-(bpdPower / teleDenom));
            robot.bsd.setPower(-(bsdPower / teleDenom));

            robot.BOW.setPower(armPower);
            robot.SOW.setPower(-armPower);

            robot.servoArm.setPower(servoArmPower);

            //robot.arm2.setPower(arm2Power);
            //robot.arm2.setPower(0.25);

            robot.servoFinger.setPosition(handPos);

            //this is not in OB, only elbowPos not the problem
            telemetry.addData("elbowPos: ", robot.arm2.getCurrentPosition());
            telemetry.addData("location", swivelLocation);
            telemetry.addData("target", swivelTarget);
            telemetry.addData("power", swivelPower);
            telemetry.update();

        }
    }
}

