package org.firstinspires.ftc.teamcode.Tamaru3.TeleOp3;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Tamaru3.Tamaru3Hardware;

/**
 * Class name: TamaruTele3
 * Class Type: tele
 * Class Function: 2 driver tele
 * Other Notes: gamepad1 is drive, gamepad 2 is arm
 */

@TeleOp
//@Disabled
public class TamaruTele3 extends LinearOpMode {
    Tamaru3Hardware robot = new Tamaru3Hardware();
    private PIDController armController;

    //the below variables are private to keep everything within this class

    //gamepad 1 (drive) variable
    private boolean polePort = false;
    private boolean poleStar = false;
    private boolean handOpen = false;
    private boolean fieldCentric = true;

    private double drivePower, strafePower, rotatePower;
    private int drivePowerDenom;

    //////////////////////////////////// ODOMETRY WHEEL CALCULATIONS //////////////////////////////////////////////
    public final double COUNTS_PER_ODO_REV = 8192, ODO_GEAR_REDUCTION = (1.0), ODO_WHEEL_DIAMETER_INCHES = 2.0,
            ODO_COUNTS_PER_INCH = ((COUNTS_PER_ODO_REV * ODO_GEAR_REDUCTION) / (ODO_WHEEL_DIAMETER_INCHES * 3.1415)),
            odoWheelGap = 12.5;//used to be 11.5

    //gamepad 2 (arm/elbow/hand) variable
    private double armPower;
    private double handPos = robot.handClosed;

    private double fpdPower, bpdPower, fsdPower, bsdPower;
    private double POWlocation, SOWlocation, BOWlocation, thetaRobot, thetaLeftJoystick, thetaFieldCentric,
            theta0, thetaRightJoystick, thetaDiscrepency, hypotenuseLeft, newStrafePower, newDrivePower;

    private double teleDenom;

    private double extendPosition = 1;
    private double turretPosition = robot.turretForward;

    // gamepad 2 (pole toucher variables)
    //changed from private to public 2/16 12:06
    public double SPTdown = .35, SPTup = .5, SPTposition = .5;
    public double PPTdown = 1, PPTup = .65, PPTposition = .63;

    public ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() {
        robot.init(hardwareMap);

        waitForStart();

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        robot.fpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.fsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.fpd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.bpd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.fsd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.bsd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.armPortI.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armPortO.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armStarI.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armStarO.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.armPortI.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.armPortO.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.armStarI.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.armStarO.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive()) {

            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            ///////////////////////////////////////////////////////////// GAMEPAD 1 //////////////////////////////////////////////////
            drivePower = -gamepad1.left_stick_y;
            strafePower = -gamepad1.left_stick_x;
            rotatePower = -gamepad1.right_stick_x;

            if (gamepad1.left_bumper) {
                handPos = robot.handClosed;
            } else if (gamepad1.right_bumper) {
                handPos = robot.handOpen;
            }

            if (gamepad1.a) {
                PPTposition = PPTdown;
            }
            if (gamepad1.b) {
                PPTposition = PPTup;
            }
            if (gamepad1.x) {
                SPTposition = SPTup;
            }
            if (gamepad1.y) {
                SPTposition = SPTdown;
            }

            /*
            if(gamepad1.left_trigger>0){
                port = true;
            } else {
                star = true;
            }

            if(port){ //add dist sensor condition
                if(dpad_up){
                    drive forward until distSensorPort < x;
                } else if(dpad_down){
                    drive forward until distSensorPort < x;
                } else if(dpad_right){
                    strafe right until distSensorPort < x;
                } else if(dpad_left){
                    strafe left until distSensorPort < x;
                }
            }

            if(star){ //add dist sensor condition
                if(dpad_up){
                        drive forward until distSensorPort < x;
                    } else if(dpad_down){
                       drive forward until distSensorPort < x;
                    } else if(dpad_right){
                        strafe right until distSensorPort < x;
                    } else if(dpad_left){
                        strafe left until distSensorPort < x;
                    }
                }
            }
            */

            /*if (gamepad1.dpad_left && !(robot.distSensorPort.getDistance(DistanceUnit.CM) < 15)) {
                drivePower = 0.5;
            }
            if (gamepad1.dpad_right && !(robot.distSensorStar.getDistance(DistanceUnit.CM) < 15)) {
                drivePower = 0.5;
            }*/

            /////////////////////////////////////////////////////////// GAMEPAD 2 ////////////////////////////////////////////////////

            armPower = -gamepad2.left_stick_y;

            if (gamepad2.left_bumper) {
                handPos = robot.handClosed;
            } else if (gamepad2.right_bumper) {
                handPos = robot.handOpen;
            }

            if (gamepad2.dpad_up) {
                turretPosition = robot.turretForward;
            } else if (gamepad2.dpad_left) {
                turretPosition = robot.turretPort;
            } else if (gamepad2.dpad_right) {
                turretPosition = robot.turretStar;
            }

            /*if(gamepad2.left_trigger>0){
                robot.servoExtend.setPosition(1);
            } else {
                robot.servoExtend.setPosition(0);
            }*/

            if (gamepad2.y) {
                //PPTposition = PPTdown;
                extendPosition = 1;
            } else if (gamepad2.b) {
                //PPTposition = PPTup;
                extendPosition = 0.9;
            } else if (gamepad2.a) {
                //SPTposition = SPTup;
                extendPosition = 0.8;
            } else if (gamepad2.x) {
                //SPTposition = SPTdown;
                extendPosition = .75;
            }

            /////////////////////////////////////////////////MATH//////////////////////////////////////////////////////////////
            POWlocation = robot.armPortO.getCurrentPosition();
            SOWlocation = robot.armStarO.getCurrentPosition();
            BOWlocation = robot.armPortI.getCurrentPosition();

            thetaRobot = (-(((POWlocation - SOWlocation) / ODO_COUNTS_PER_INCH) / odoWheelGap));// we sure about this negative?

            thetaLeftJoystick = Math.atan2((gamepad1.left_stick_x), (-gamepad1.left_stick_y));//-
            //thetaRightJoystick = Math.atan2((-gamepad1.right_stick_x), (gamepad1.right_stick_y)); //check signs
            thetaFieldCentric = thetaRobot - thetaLeftJoystick + theta0; // -(Math.PI/2)

            thetaDiscrepency = thetaRightJoystick - thetaRobot + theta0 + Math.PI;
            //hypotenuseRight = Math.sqrt((gamepad1.right_stick_x * gamepad1.right_stick_x)+(gamepad1.right_stick_y * gamepad1.right_stick_y));
            //rotatePower = hypotenuseRight*(Math.sin(thetaDiscrepency / 2));

            hypotenuseLeft = Math.sqrt((gamepad1.left_stick_x * gamepad1.left_stick_x)+(gamepad1.left_stick_y * gamepad1.left_stick_y));
            newStrafePower = (hypotenuseLeft * Math.cos(thetaFieldCentric));
            newDrivePower =  -(hypotenuseLeft * Math.sin(thetaFieldCentric));

            ///////////////////////////////////////////////////////// MOTOR POWERS ////////////////////////////////////////////////////

            if (gamepad1.left_trigger > 0) {
                drivePowerDenom = 2;
            } else {
                drivePowerDenom = 1;
            }

            if (currentGamepad1.back && !previousGamepad1.back){
                fieldCentric = !fieldCentric;
            }

            /*if(!fieldCentric) {
                fpdPower = (drivePower - strafePower - rotatePower) / drivePowerDenom;
                bpdPower = (drivePower + strafePower - rotatePower) / drivePowerDenom;
                fsdPower = (drivePower + strafePower + rotatePower) / drivePowerDenom;
                bsdPower = (drivePower - strafePower + rotatePower) / drivePowerDenom;
            } else {
                fpdPower = (newDrivePower + newStrafePower + rotatePower) / drivePowerDenom;
                bpdPower = (newDrivePower - newStrafePower + rotatePower) / drivePowerDenom;
                fsdPower = (newDrivePower - newStrafePower - rotatePower) / drivePowerDenom;
                bsdPower = (newDrivePower + newStrafePower - rotatePower) / drivePowerDenom;
            }*/

            fpdPower = (drivePower - strafePower - rotatePower) / drivePowerDenom;
            bpdPower = (drivePower + strafePower - rotatePower) / drivePowerDenom;
            fsdPower = (drivePower + strafePower + rotatePower) / drivePowerDenom;
            bsdPower = (drivePower - strafePower + rotatePower) / drivePowerDenom;

            /*fpdPower = (newDrivePower + newStrafePower + rotatePower) / drivePowerDenom;
            bpdPower = (newDrivePower - newStrafePower + rotatePower) / drivePowerDenom;
            fsdPower = (newDrivePower - newStrafePower - rotatePower) / drivePowerDenom;
            bsdPower = (newDrivePower + newStrafePower - rotatePower) / drivePowerDenom;*/

            teleDenom = Math.max(Math.max(Math.abs(fpdPower), Math.abs(bpdPower)), Math.max(Math.abs(fsdPower), Math.abs(bsdPower)));

            robot.fpd.setPower(fpdPower);
            robot.fsd.setPower(fsdPower);
            robot.bpd.setPower(bpdPower);
            robot.bsd.setPower(bsdPower);

            robot.armPortI.setPower(armPower);
            robot.armPortO.setPower(armPower);
            robot.armStarI.setPower(armPower);
            robot.armStarO.setPower(armPower);

            robot.servoExtend.setPosition(extendPosition);

            robot.servoTurret.setPosition(turretPosition);

            robot.servoHand.setPosition(handPos);

            //robot.servoPoleToucherStar.setPosition(SPTposition);
            //robot.servoPoleToucherPort.setPosition(PPTposition);

            ////////////////////////////////////////////////OTHER/////////////////////////////////////////////////////////////
            if ((robot.distSensorHand.getDistance(DistanceUnit.CM) < 4) && !(gamepad2.left_bumper || gamepad2.right_bumper) && !(gamepad1.left_bumper || gamepad1.right_bumper)) {
                handPos = robot.handClosed;
            }

            /*if (robot.touchSensorPort.isPressed()) {
                polePort = true;
            }
            if (robot.touchSensorStar.isPressed()) {
                poleStar = true;
            }*/

            if (handPos == robot.handOpen) {
                handOpen = true;
                polePort = false;
                poleStar = false;
            } else {
                handOpen = false;
            }

            /*if(gamepad2.left_trigger>0){
                extendPosition = 1;
            } else {
                extendPosition = 0;
            }*/

            /*if ((polePort) && (!handOpen)) {
                turretPosition = robot.turretPort;
                extendPosition = .65;//.75
            } else if ((poleStar) && (!handOpen)) {
                turretPosition = robot.turretStar;
                extendPosition = .25;//.75, .7
            } else if ((!polePort) && (!poleStar) && (!gamepad2.dpad_right) && (!gamepad2.dpad_right)) {
                turretPosition = robot.turretForward;
                extendPosition = 0;
            }*/

            /*if (robot.touchSensorPort.isPressed() || robot.touchSensorStar.isPressed()) {
                robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
            } else {
                robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_GREEN);
            }*/

            telemetry.addData("armPortO", robot.armPortO.getCurrentPosition());
            telemetry.addData("armStarO", robot.armStarO.getCurrentPosition());
            telemetry.addData("armPortI", robot.armPortI.getCurrentPosition());
            telemetry.addData("armStarI", robot.armStarI.getCurrentPosition());
            telemetry.update();
        }
    }
}