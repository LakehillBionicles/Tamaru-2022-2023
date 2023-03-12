package org.firstinspires.ftc.teamcode.Tamaru2.TeleOp2.Current;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Tamaru2.BaseClasses.Tamaru2Hardware;


@TeleOp
//@Disabled

//////////////////////gamepad1 is drive; gamepad 2 is arm/hand/pre-set distances//////////////////////

public class fancyTeleOp extends LinearOpMode {
    Tamaru2Hardware robot = new Tamaru2Hardware();
    private PIDController armController;

    public static double pArm = .03, iArm = 0.001, dArm = 0.0005;
    public static double fArm = 0;

    //the below variables are private to keep everything within this class

    //gamepad 1 (drive) variable

    private boolean polePort = false;
    private boolean poleStar = false;
    private boolean handOpen = false;
    private boolean scored = false;
    private boolean portPoleToucherDown = false;
    private boolean starPoleToucherDown = false;

    private double drivePower;
    private double strafePower;
    private double rotatePower;
    private int drivePowerDenom;

    //////////////////////////////////// ODOMETRY WHEEL CALCULATIONS //////////////////////////////////////////////
    public final double COUNTS_PER_ODO_REV = 8192;
    public final double ODO_GEAR_REDUCTION = (1.0); // This is < 1.0 if geared UP
    public final double ODO_WHEEL_DIAMETER_INCHES = 2.0;  // For figuring circumference
    public final double ODO_COUNTS_PER_INCH = ((COUNTS_PER_ODO_REV * ODO_GEAR_REDUCTION) /
            (ODO_WHEEL_DIAMETER_INCHES * 3.1415));

    public final double odoWheelGap = 12.5;//used to be 11.5

    private double POWlocation;
    private double SOWlocation;
    private double BOWlocation;

    //gamepad 2 (arm/elbow/hand) variable
    private double armPower;
    private double elbowPower;
    private double servoArmPower;
    private double handPos = 0.5;


    private double fpder;
    private double bpdPower;
    private double fsdPower;
    private double bsdPower;

    private double teleDenom;

    private double extendPosition = 0;
    private double turretPosition = robot.turretForward;

    // gamepad 2 (pole toucher variables)
    //changed from private to public 2/16 12:06
    public double SPTdown=.35;
    public double SPTup=.5;
    public double SPTposition=.5;
    public double PPTdown=1;
    public double PPTup=.65;
    public double PPTposition=.63;



    public ElapsedTime runtime = new ElapsedTime();


    public void runOpMode() {
        robot.init(hardwareMap);
        armController = new PIDController(pArm, iArm, dArm);

        waitForStart();


        robot.fpd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.bpd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.fsd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.bsd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.armPort.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armStar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.armPort.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.armStar.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.extendEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        while (opModeIsActive()) {

            ///////////////////////////////////////////////////////////// GAMEPAD 1 //////////////////////////////////////////////////
            drivePower = gamepad1.left_stick_y;
            strafePower = -gamepad1.left_stick_x;
            rotatePower = -gamepad1.right_stick_x;

            if (gamepad1.left_bumper) {
                handPos = robot.handClosed;
            } else if (gamepad1.right_bumper) {
                handPos = robot.handOpen;
            }

            if (gamepad1.a){
                PPTposition=PPTdown;
            }
            if (gamepad1.b){
                PPTposition=PPTup;
            }
            if (gamepad1.x){
                SPTposition=SPTup;
            }
            if(gamepad1.y){
                SPTposition=SPTdown;
            }

            if(gamepad1.dpad_left&&!(robot.distSensorPort.getDistance(DistanceUnit.CM)<15)){
                drivePower = 0.5;
            }
            if(gamepad1.dpad_right&&!(robot.distSensorStar.getDistance(DistanceUnit.CM)<15)){
                drivePower = 0.5;
            }




            /////////////////////////////////////////////////////////// GAMEPAD 2 ////////////////////////////////////////////////////

            armPower = -gamepad2.left_stick_y;

            if (gamepad2.left_bumper) {
                handPos = robot.handClosed;
            } else if (gamepad2.right_bumper) {
                handPos = robot.handOpen;
            }

            /*if (gamepad2.left_trigger>0) {
                turretPosition = robot.turretPort;
            } else if (gamepad2.right_trigger>0) {
                turretPosition = robot.turretStar;
            } else {
                turretPosition = robot.turretForward;
            }*/

            if (gamepad2.dpad_up) {
                turretPosition = robot.turretForward;
            } else if (gamepad2.dpad_left) {
                turretPosition = robot.turretPort;
            } else if (gamepad2.dpad_right) {
                turretPosition = robot.turretStar;
            }

            /*if(gamepad2.a){
                extendPosition += .05;
            } else if(gamepad2.b){
                extendPosition -= .05;
            }*/

            if (gamepad2.a){
                PPTposition=PPTdown;
            }
            if (gamepad2.b){
                PPTposition=PPTup;
            }
            if (gamepad2.x){
                SPTposition=SPTup;
            }
            if(gamepad2.y){
                SPTposition=SPTdown;
            }

            ///////////////////////////////////////////////////////// MOTOR POWERS ////////////////////////////////////////////////////

            if(gamepad1.left_trigger > 0){
                drivePowerDenom = 3;
            } else if(gamepad1.right_trigger > 0){
                drivePowerDenom = 1;
            } else{
                drivePowerDenom = 2;
            }


            fpder = drivePower + strafePower + rotatePower/2;
            bpdPower = drivePower - strafePower + rotatePower/2;
            fsdPower = drivePower - strafePower - rotatePower/2;
            bsdPower = drivePower + strafePower - rotatePower/2;

            teleDenom = Math.max(Math.max(Math.abs(fpder), Math.abs(bpdPower)), Math.max(Math.abs(fsdPower), Math.abs(bsdPower)));

            robot.fpd.setPower((fpder / teleDenom)/2);
            robot.fsd.setPower((fsdPower / teleDenom)/2);
            robot.bpd.setPower(((bpdPower / teleDenom))/2);
            robot.bsd.setPower(((bsdPower / teleDenom))/2);

            robot.armPort.setPower(armPower);
            robot.armStar.setPower(armPower);

            robot.servoExtend.setPosition(extendPosition);

            robot.servoTurret.setPosition(turretPosition);

            robot.servoHand.setPosition(handPos);

            robot.servoPoleToucherStar.setPosition(SPTposition);
            robot.servoPoleToucherPort.setPosition(PPTposition);

            ////////////////////////////////////////////////OTHER/////////////////////////////////////////////////////////////
            if ((robot.distSensorHand.getDistance(DistanceUnit.CM)<4) && !(gamepad2.left_bumper || gamepad2.right_bumper) && !(gamepad1.left_bumper || gamepad1.right_bumper)){
                handPos = robot.handClosed;
            }

            /*if(portPoleToucherDown==true){
                PPTposition = PPTdown;
            } else {
                PPTposition = PPTup;
            }

            if(starPoleToucherDown==true){
                SPTposition = SPTdown;
            } else {
                SPTposition = SPTup;
            }*/

            if(robot.touchSensorPort.isPressed()){
                polePort = true;
            }
            if(robot.touchSensorStar.isPressed()){
                poleStar = true;
            }
            if(handPos==robot.handOpen){
                handOpen = true;
                polePort = false;
                poleStar = false;
            } else {
                handOpen = false;
            }

            if((polePort) && (!handOpen)){
                turretPosition = robot.turretPort;
                extendPosition = .65;//.75
            } else if((poleStar) && (!handOpen)){
                turretPosition = robot.turretStar;
                extendPosition = .25;//.75, .7
            } else if((!polePort)&&(!poleStar)&&(!gamepad2.dpad_right)&&(!gamepad2.dpad_right)){
                turretPosition = robot.turretForward;
                extendPosition = 0;
            }

            if (robot.touchSensorPort.isPressed() || robot.touchSensorStar.isPressed()){
                robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
            } else {
                robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_GREEN);
            }


            telemetry.addData("red", robot.colorSensorStar.red());
            telemetry.addData("blue", robot.colorSensorStar.blue());
            telemetry.addData("green", robot.colorSensorStar.green());
            telemetry.update();
        }
    }
    public void PIDArmControl(double targetArm, double timeout){
        armController.setPID(pArm, iArm, dArm);
        armController.setSetPoint(targetArm);
        armController.setTolerance(20);

        double robotArmPort = robot.armPort.getCurrentPosition();
        double robotArmStar = robot.armStar.getCurrentPosition();
        double robotArm = (robotArmPort+robotArmStar)/2;

        resetRuntime();
        while((!armController.atSetPoint()) && getRuntime()<timeout){
            robotArmPort = robot.armPort.getCurrentPosition();
            robotArmStar = robot.armStar.getCurrentPosition();
            robotArm = (robotArmPort+robotArmStar)/2;

            double pidArm = armController.calculate(robotArm, armController.getSetPoint());

            double ffArm = targetArm * fArm;

            double armPower = (pidArm + ffArm);

            robot.armPort.setPower(armPower);
            robot.armStar.setPower(armPower);

        }
        robot.armPort.setPower(0);
        robot.armStar.setPower(0);
    }
}
