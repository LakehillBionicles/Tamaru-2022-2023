package org.firstinspires.ftc.teamcode.Tamaru2.TeleOp2.Old;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Tamaru2.BaseClasses.Tamaru2Hardware;
import org.firstinspires.ftc.teamcode.Tamaru2.BaseClasses.executionClass;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Tamaru2.BaseClasses.Tamaru2Hardware;




@TeleOp
@Disabled

    /*
    DRIVE
    EXTEND
    HAND
    TURRET
    ARM
    EXECUTION
     */

public class uglyTamaru2TeleOp extends LinearOpMode {

    Tamaru2Hardware robot = new Tamaru2Hardware();

    //changed all from private to public 2/5 3:48
    public double fpdPOWPower = 0;
    public double bpdBOWPower = 0;
    public double fsdSOWPower = 0;
    public double bsdPower = 0;
    public double extendPower = 0;
    public double handPosition = handClosed;
    public double turretPosition = turretForward;
    public int armTargetPosition = 0;
    public double armPower = 0;
    public boolean armToHeight = true;
    public boolean fieldCentric = true;
    public double drivePower;
    public double strafePower;
    public double rotatePower;
    public double driveDenom = 1;
    public String mode = "tele";


    public ElapsedTime runtime = new ElapsedTime();

    ////////////////////////////////CALCULATIONS VARIABLES//////////////////////////////////////////////////////
    static final double FEET_PER_METER = 3.28084;

    ////////////////////////////////TELEMETRY////////////////////////////////////////////////
    public boolean whereAmITelemetry = false;
    public boolean odoCalculationsTelemetry = false;
    public boolean senseColorTelemetry = true;

    ////////////////////////////////TURRET TARGETS////////////////////////////////////////////////
    public static final double turretForward = .5;//not checked
    public static final double turretPort = 0;//not checked
    public static final double turretStar = 1;//not checked

    ////////////////////////////////HAND TARGETS////////////////////////////////////////////////
    public static final double handOpen = 1;
    public static final double handClosed = .7;

    ////////////////////////////////ARM TARGETS////////////////////////////////////////////////
    public static final int downArmTarget = 0;//not checked
    public static final int lowArmTarget = 0;//not checked
    public static final int midArmTarget = 0;//not checked
    public static final int highArmTarget = 0;//not checked

    //////////////////////////////// DRIVE TARGET VALUES ////////////////////////////////////////////////
    public double newTargetTheta = 0;
    public double newTargetX = 0;
    public double newTargetY = 0;

    //////////////////////////////////// ODOMETRY WHEEL CALCULATIONS //////////////////////////////////////////////
    public final double COUNTS_PER_ODO_REV = 8192;
    public final double ODO_GEAR_REDUCTION = (1.0);
    public final double ODO_WHEEL_DIAMETER_INCHES = 2.0;
    public final double ODO_COUNTS_PER_INCH = ((COUNTS_PER_ODO_REV * ODO_GEAR_REDUCTION) /
            (ODO_WHEEL_DIAMETER_INCHES * 3.1415));

    public final double odoWheelGap = 12.5; //used to be 11.5

    //////////////////////////////// LOCATIONS ////////////////////////////////////////////////
    public double POWlocation = 0;
    public double SPOWlocation = 0;
    public double SOWlocation = 0;
    public double BOWlocation = 0;

    public double robotTheta = (-(((POWlocation - SOWlocation) / ODO_COUNTS_PER_INCH) / odoWheelGap));
    public double robotX = (BOWlocation / ODO_COUNTS_PER_INCH);
    public double robotXThetaCorrect = ((BOWlocation / ODO_COUNTS_PER_INCH) - (5 * robotTheta));
    public double robotY = (((POWlocation + SOWlocation) / 2) / ODO_COUNTS_PER_INCH);

    //////////////////////////////// ODO POWERS ////////////////////////////////////////////////
    public double thetaPower = 0;
    public double xPower = 0;
    public double yPower = 0;

    public double fpdPOWOdoPower = 0;
    public double bpdBOWOdoPower = 0;
    public double fsdSOWOdoPower = 0;
    public double bsdOdoPower = 0;

    //////////////////////////////// ODO ERRORS ////////////////////////////////////////////////
    public double xError = (newTargetX - robotX);
    public double yError = (newTargetY - robotY);
    public double thetaError = (newTargetTheta - robotTheta);

    public double lastThetaError = 0;
    public double lastXError = 0;
    public double lastYError = 0;

    //////////////////////////////// DERIV VARIABLES ////////////////////////////////////////////////
    public double thetaDeriv = 0;
    public double xDeriv = 0;
    public double yDeriv = 0;

    //////////////////////////////// INTEGRAL VARIABLES ////////////////////////////////////////////////
    public double thetaIntegralSum = 0;
    public double xIntegralSum = 0;
    public double yIntegralSum = 0;

    public double integralSumLimit = 0.25;

    //////////////////////////////// PID VARIABLES ////////////////////////////////////////////////
    public double Kp = .33;
    public double Kd = .33;
    public double Ki = .33;

    //////////////////////////////// MISC VARIABLES ////////////////////////////////////////////////
    public double odoTime = 0;

    public double denominator = Math.max(Math.abs(yPower) + Math.abs(xPower) + Math.abs(thetaPower), 1);
    public double denominator2 = 0;

    public boolean odoCalculationsFinished = true;

    /*public int SOWPos = 0;
    public int POWPos = 0;
    public int BOWPos = 0;*/




    /////////////////////////////////////FIELD CENTRIC VARIABLES//////////////////////////////////////////
    public double drivePowerDenom;

    public double hypotenuseLeft;
    public double thetaFieldCentric;
    public double thetaRobot;
    public double thetaLeftJoystick;
    public double thetaRightJoystick;
    public double thetaDiscrepency;
    public double newX;
    public double newY;
    public double theta0;

    public double fpdPOWFCPower = 0;
    public double bpdBOWFCPower = 0;
    public double fsdSOWFCPower = 0;
    public double bsdFCPower = 0;

    public boolean fieldCentricUp = false;
    public boolean fieldCentricDown = false;
    public boolean fieldCentricLeft = false;
    public boolean fieldCentricRight = false;

    //////////////////////////////////// WHEEL CALCULATIONS //////////////////////////////////////////////
    static final double COUNTS_PER_MOTOR_REV = 560;
    static final double DRIVE_GEAR_REDUCTION = (0.11111); // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.0;  // For figuring circumference
    static final double COUNTS_PER_INCH = 1.2 * 4 * ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415));


    static final String sleeveColor = "";


    public void runOpMode() {

        robot.init(hardwareMap);

        waitForStart();

        resetEverything();

        while (opModeIsActive()) {


            ///////////////////////////////////////DRIVE////////////////////////////////////////////////////////
            drivePower = gamepad1.left_stick_y;
            strafePower = -gamepad1.left_stick_x;
            rotatePower = -gamepad1.right_stick_x;

            if(gamepad1.dpad_up){
                fieldCentricUp = true;
                fieldCentricDown = false;
                fieldCentricLeft = false;
                fieldCentricRight = false;
            } else if(gamepad1.dpad_down){
                fieldCentricUp = false;
                fieldCentricDown = true;
                fieldCentricLeft = false;
                fieldCentricRight = false;
            } else if(gamepad1.dpad_left){
                fieldCentricUp = false;
                fieldCentricDown = false;
                fieldCentricLeft = true;
                fieldCentricRight = false;
            } else if(gamepad1.dpad_right){
                fieldCentricUp = false;
                fieldCentricDown = false;
                fieldCentricLeft = false;
                fieldCentricRight = true;
            }

            if(gamepad1.left_trigger>0){
                driveDenom = .5;
            } else {
                driveDenom = 1;
            }

            if(gamepad1.a){
                fieldCentric = false;
            } else if(gamepad1.b){
                fieldCentric = true;
            }

            if(fieldCentric) {
                fpdPOWPower = fpdPOWFCPower;
                bpdBOWPower = bpdBOWFCPower;
                fsdSOWPower = fpdPOWFCPower;
                bsdPower = fpdPOWFCPower;
            } else {
                fpdPOWPower = drivePower + strafePower + rotatePower;
                bpdBOWPower = drivePower - strafePower + rotatePower;
                fsdSOWPower = drivePower - strafePower - rotatePower;
                bsdPower = drivePower + strafePower - rotatePower;
            }

            ///////////////////////////////////////EXTEND////////////////////////////////////////////////////////
            if(gamepad2.right_stick_y < 0){
                extendPower = 1;
                //robot.servoExtend.setPower(1);
            } else if(gamepad2.right_stick_y > 0){
                extendPower = -1;
            } else{
                extendPower = 0;
            }

            ///////////////////////////////////////HAND////////////////////////////////////////////////////////
            if (gamepad2.left_bumper || gamepad1.left_bumper) {
                handPosition = handClosed;
            } else if (gamepad2.right_bumper || gamepad1.right_bumper) {
                handPosition = handOpen;
            }

            ///////////////////////////////////////TURRET////////////////////////////////////////////////////////
            if(gamepad2.dpad_up){
                turretPosition = turretForward;
            } else if (gamepad2.dpad_right){
                turretPosition = turretStar;
            } else if (gamepad2.dpad_left){
                turretPosition = turretPort;
            }

            ///////////////////////////////////////ARM////////////////////////////////////////////////////////
            if(gamepad2.left_trigger>0){
                armToHeight = false;
            } else if(gamepad2.right_trigger>0){
                armToHeight = true;
            }

            if(gamepad2.a && armToHeight){
                armTargetPosition = downArmTarget;
            } else if(gamepad2.x && armToHeight){
                armTargetPosition = lowArmTarget;
            } else if(gamepad2.y && armToHeight){
                armTargetPosition = midArmTarget;
            } else if(gamepad2.b && armToHeight){
                armTargetPosition = highArmTarget;
            }

            if(!armToHeight){
                if(gamepad2.left_stick_y<0){
                    armPower = -gamepad2.left_stick_y;
                } else if(gamepad2.left_stick_y>0){
                    armPower = -gamepad2.left_stick_y/2;
                }
            }

            ///////////////////////////////////////EXECUTION////////////////////////////////////////////////////////
            //execution(fpdPOWPower, bpdBOWPower, fsdSOWPower, bsdPower, driveDenom, extendPower, handPosition, turretPosition, armTargetPosition, armPower, armToHeight, mode);
            if(mode=="tele") {
                fieldCentricCalculations();

                robot.fpd.setPower(fpdPOWPower/driveDenom);
                robot.bpd.setPower(bpdBOWPower/driveDenom);
                robot.fsd.setPower(fsdSOWPower/driveDenom);
                robot.bsd.setPower(bsdPower/driveDenom);
            } else if(mode=="auto") {
                robot.fpd.setPower(fpdPOWOdoPower);
                robot.bpd.setPower(bpdBOWOdoPower);
                robot.fsd.setPower(fsdSOWOdoPower);
                robot.bsd.setPower(bsdOdoPower);
            }

            //robot.servoExtend.setPower(extendPower);

            robot.servoHand.setPosition(handPosition);
            robot.servoTurret.setPosition(turretPosition);

            if (armToHeight == true){
                robot.armPort.setTargetPosition(armTargetPosition);
                robot.armPort.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.armPort.setPower(armPower);
                robot.armStar.setPower(armPower);
            } else if (armToHeight == false) {
                robot.armPort.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.armStar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.armPort.setPower(armPower);
                robot.armStar.setPower(armPower);
            }

        }

    }

    /////////////////////////CALCULATIONS BASE/////////////////////////////////////////////
////////////////////////////ODO WHEELS////////////////////////////////////////////
    public void odoWhereAmI() {
        POWlocation = robot.fpd.getCurrentPosition();
        SOWlocation = robot.fsd.getCurrentPosition();
        BOWlocation = robot.bpd.getCurrentPosition();

        robotTheta = (-(((POWlocation - SOWlocation) / ODO_COUNTS_PER_INCH) / odoWheelGap));
        robotX = (BOWlocation / ODO_COUNTS_PER_INCH) - (5 * robotTheta);
        robotY = (((POWlocation + SOWlocation) / 2) / ODO_COUNTS_PER_INCH);

        xError = (newTargetX - robotX);
        yError = (newTargetY - robotY);
        thetaError = (newTargetTheta - robotTheta);

        if (whereAmITelemetry) {
            telemetry.addData("robotTheta:", robotTheta);
            telemetry.addData("robotX:", robotX);
            telemetry.addData("robotY:", robotY);

            telemetry.addData("thetaError:", thetaError);
            telemetry.addData("xError:", xError);
            telemetry.addData("yError:", yError);

            telemetry.update();
        }
    }

    public void odoPowerCalculations(double thetaTarget, double xTarget, double yTarget, double rotPower, double linXPower, double linYPower, double rotTol, double xTol, double yTol) {
        odoWhereAmI();

        while (Math.abs(thetaError) > Math.toRadians(rotTol) || Math.abs(xError) > xTol || Math.abs(yError) > yTol) {
            odoCalculationsFinished = false;

            odoTime = getRuntime();

            odoWhereAmI();

            newTargetTheta = Math.toRadians(thetaTarget) + robotTheta;
            newTargetX = ((xTarget) + robotX);
            newTargetY = ((yTarget) + robotY);

            denominator2 = Math.max(Math.abs(yPower) + Math.abs(xPower) + Math.abs(thetaPower), 1);

            fpdPOWOdoPower = (((linYPower * (yPower)) + (linXPower * (xPower)) + (rotPower * thetaPower)) / denominator);
            bpdBOWOdoPower = (((linYPower * (yPower)) - (linXPower * (xPower)) + (rotPower * thetaPower)) / denominator);
            fsdSOWOdoPower = (((linYPower * (yPower)) - (linXPower * (xPower)) - (rotPower * thetaPower)) / denominator);
            bsdOdoPower = (((linYPower * (yPower)) + (linXPower * (xPower)) - (rotPower * thetaPower)) / denominator);

            if (Math.abs(thetaError) > Math.toRadians(rotTol)) {
                thetaDeriv = ((thetaError - lastThetaError) / odoTime);
                thetaIntegralSum = (thetaIntegralSum + (thetaError * odoTime));
                if (thetaIntegralSum > integralSumLimit) {
                    thetaIntegralSum = integralSumLimit;
                }
                if (thetaIntegralSum < -integralSumLimit) {
                    thetaIntegralSum = -integralSumLimit;
                }
                thetaPower = -((Kd * thetaDeriv) + (Ki * thetaIntegralSum) + (Kp * Math.signum(thetaError)));
            } else {
                thetaPower = 0;
            }

            if (Math.abs(xError) > xTol) {
                xDeriv = ((xError - lastXError) / odoTime);
                xIntegralSum = (xIntegralSum + (xError * odoTime));
                if (xIntegralSum > integralSumLimit) {
                    xIntegralSum = integralSumLimit;
                }
                if (xIntegralSum < -integralSumLimit) {
                    xIntegralSum = -integralSumLimit;
                }
                xPower = ((Kd * xDeriv) + (Ki * xIntegralSum) + (Kp * Math.signum(xError)));
            } else {
                xPower = 0;
            }

            if (Math.abs(yError) > yTol) {
                yDeriv = ((yError - lastYError) / odoTime);
                yIntegralSum = (yIntegralSum + (yError * odoTime));
                if (yIntegralSum > integralSumLimit) {
                    yIntegralSum = integralSumLimit;
                }
                if (yIntegralSum < -integralSumLimit) {
                    yIntegralSum = -integralSumLimit;
                }
                yPower = ((Kd * yDeriv) + (Ki * yIntegralSum) + (Kp * Math.signum(yError)));
            } else {
                yPower = 0;
            }

        }

        if (odoCalculationsTelemetry) {
            telemetry.addData("robotTheta:", robotTheta);
            telemetry.addData("robotX:", robotX);
            telemetry.addData("robotY:", robotY);

            telemetry.addData("newTargetTheta:", newTargetTheta);
            telemetry.addData("newTargetX:", newTargetX);
            telemetry.addData("newTargetY:", newTargetY);

            telemetry.addData("thetaError:", thetaError);
            telemetry.addData("xError:", xError);
            telemetry.addData("yError:", yError);

            telemetry.addData("thetaPower:", thetaPower);
            telemetry.addData("xPower:", xPower);
            telemetry.addData("yPower:", yPower);

            telemetry.addData("yay", "made it");
            telemetry.update();
        }

        odoCalculationsFinished = true;
    }

    //////////////////////////FIELD CENTRIC CALCULATIONS/////////////////////////////
    public void fieldCentricReset() {
        if (fieldCentricUp/*== true*/) {
            robot.fpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.bpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.fsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.fpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.fsd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            theta0 = 0;
        } else if (fieldCentricDown) {
            robot.fpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.bpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.fsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.fpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.fsd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            theta0 = Math.PI;
        } else if (fieldCentricLeft) { //flipped left and right 1/27 7:10
            robot.fpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.bpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.fsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.fpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.fsd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            theta0 = (Math.PI) / 2;
        } else if (fieldCentricRight) {
            robot.fpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.bpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.fsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.fpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.fsd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            theta0 = (3 * Math.PI) / 2;
        }
    }

    public void fieldCentricCalculations() {
        fieldCentricReset();

        POWlocation = robot.fpd.getCurrentPosition();
        SOWlocation = robot.fsd.getCurrentPosition();
        BOWlocation = robot.bpd.getCurrentPosition();

        thetaRobot = (-(((POWlocation - SOWlocation) / ODO_COUNTS_PER_INCH) / odoWheelGap));// we sure about this negative?

        thetaLeftJoystick = Math.atan2((-gamepad1.left_stick_x), (gamepad1.left_stick_y));
        //thetaRightJoystick = Math.atan2((-gamepad1.right_stick_x), (gamepad1.right_stick_y)); //check signs
        thetaFieldCentric = thetaRobot - thetaLeftJoystick + theta0; // -(Math.PI/2)

        thetaDiscrepency = thetaRightJoystick - thetaRobot + theta0 + Math.PI;
        //hypotenuseRight = Math.sqrt((gamepad1.right_stick_x * gamepad1.right_stick_x)+(gamepad1.right_stick_y * gamepad1.right_stick_y));
        //rotatePower = hypotenuseRight*(Math.sin(thetaDiscrepency / 2));

        hypotenuseLeft = Math.sqrt((gamepad1.left_stick_x * gamepad1.left_stick_x) + (gamepad1.left_stick_y * gamepad1.left_stick_y));
        newX = (hypotenuseLeft * Math.sin(thetaFieldCentric));
        newY = -(hypotenuseLeft * Math.cos(thetaFieldCentric));

        fpdPOWFCPower = ((-1 * (newY + newX + rotatePower)) / drivePowerDenom);//signs not checked
        bpdBOWFCPower = (((newY - newX + rotatePower)) / drivePowerDenom);//signs not checked
        fsdSOWFCPower = ((-1 * (newY - newX - rotatePower)) / drivePowerDenom);//signs not checked
        bsdFCPower = (((newY + newX - rotatePower)) / drivePowerDenom);//signs not checked
    }

    /////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////START UP///////////////////////////////////////
    public void startUp() {
        robot.init(hardwareMap);


        ////////////reset encoders right here - added to make PID work
        robot.fpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.fsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);

        setMotorDir();
        //setMotorDirStrafe();

        robot.fpd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.bpd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.fsd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.bsd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.fpd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.bpd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.fsd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.bsd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.fpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.fsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.fpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.fsd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bsd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //initVuforia();

        //initTfod();

        telemetry.addData("Vision is Ready", ")");
        telemetry.update();


    }

    public void setMotorDir () {
        robot.fpd.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.bpd.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.fsd.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.bsd.setDirection(DcMotorSimple.Direction.FORWARD);

    }

    public void setMotorDirStrafe () {
        robot.fpd.setDirection(DcMotorSimple.Direction.REVERSE);//not checked
        robot.bpd.setDirection(DcMotorSimple.Direction.FORWARD);//not checked
        robot.fsd.setDirection(DcMotorSimple.Direction.REVERSE);//not checked
        robot.bsd.setDirection(DcMotorSimple.Direction.FORWARD);//not checked
    }

    public void resetEverything() {
        robot.fpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.fsd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bsd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.fpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.fsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.armPort.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armStar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.fpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.fsd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bsd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.armPort.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.armStar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    ////////////////////////////////////SENSE COLORS/////////////////////////////////?????????????????????????????????????
    public void senseColorTelemetry(){ //drive until see not green
        while (opModeIsActive()){
            if(senseColorTelemetry) {
                telemetry.addData("redStar", robot.colorSensorFront.red());
                telemetry.addData("greenStar", robot.colorSensorFront.green());
                telemetry.addData("blueStar", robot.colorSensorFront.blue());

                telemetry.addData("redPort", robot.colorSensorPort.red());
                telemetry.addData("greenPort", robot.colorSensorPort.green());
                telemetry.addData("bluePort", robot.colorSensorPort.blue());

                telemetry.update();
            }

        }
    }

    public String senseColorsFront () {
        String colorStar = "blank";

        while (opModeIsActive() && colorStar.equals("blank")) {
            if (robot.colorSensorFront.red() > (robot.colorSensorFront.blue()) && robot.colorSensorFront.red() > (robot.colorSensorFront.green())) {
                colorStar = "red";
                telemetry.addData("i see red", " ");
                telemetry.update();
                colorStar = "red";
                //sleeveColor.equals(red);

            } else if (robot.colorSensorFront.blue() > (robot.colorSensorFront.red()) && robot.colorSensorFront.blue() > (robot.colorSensorFront.green())) {
                colorStar = "blue";
                telemetry.addData("i see blue", " ");
                telemetry.update();
                colorStar = "blue";

            } else if (robot.colorSensorFront.green() > (robot.colorSensorFront.red()) && robot.colorSensorFront.green() > (robot.colorSensorFront.blue())) {
                colorStar = "green";
                telemetry.addData("i see green", " ");
                telemetry.update();
                colorStar = "green";

            } else {
                telemetry.addData("i see nothing", " ");
                telemetry.update();
                colorStar = "no go";

            }

        }
        return colorStar;
    }

    public String senseColorsPort(){
        String colorPort = "blank";

        while (opModeIsActive()&& colorPort.equals("blank")){

            if (robot.colorSensorPort.red() > (robot.colorSensorPort.blue()) && robot.colorSensorPort.red() > (robot.colorSensorPort.green())){
                colorPort = "red";
                telemetry.addData("i see red", " ");
                telemetry.update();
                colorPort ="red";


            } else if (robot.colorSensorPort.blue() > (robot.colorSensorPort.red()) && robot.colorSensorPort.blue() > (robot.colorSensorPort.green())){
                colorPort = "blue";
                telemetry.addData("i see blue", " ");
                telemetry.update();
                colorPort ="blue";

            } else if (robot.colorSensorPort.green() > (robot.colorSensorPort.red()) && robot.colorSensorPort.green() > (robot.colorSensorPort.blue())){
                colorPort = "green";
                telemetry.addData("i see green", " ");
                telemetry.update();
                colorPort = "green";

            }else {
                telemetry.addData("i see nothing", " ");
                telemetry.update();
                colorPort = "no go";
            }
        }

        return colorPort;
    }

    ///////////////////////////////////////EXECUTION CLASS////////////////////////////////////////////////////////
    public void execution(double fpdPOWPower, double bpdBOWPower, double fsdSOWPower, double bsdPower, double driveDenom, double extendPower,
                          double handPosition, double turretPosition, int armTargetPosition, double armPower, boolean armToHeight, String mode){
        if(mode=="tele") {
            fieldCentricCalculations();

            robot.fpd.setPower(fpdPOWPower/driveDenom);
            robot.bpd.setPower(bpdBOWPower/driveDenom);
            robot.fsd.setPower(fsdSOWPower/driveDenom);
            robot.bsd.setPower(bsdPower/driveDenom);
        } else if(mode=="auto") {
            robot.fpd.setPower(fpdPOWOdoPower);
            robot.bpd.setPower(bpdBOWOdoPower);
            robot.fsd.setPower(fsdSOWOdoPower);
            robot.bsd.setPower(bsdOdoPower);
        }

        //robot.servoExtend.setPower(extendPower);

        robot.servoHand.setPosition(handPosition);
        robot.servoTurret.setPosition(turretPosition);

        if (armToHeight == true){
            robot.armPort.setTargetPosition(armTargetPosition);
            robot.armPort.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.armPort.setPower(armPower);
            robot.armStar.setPower(armPower);
        } else if (armToHeight == false) {
            robot.armPort.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.armStar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.armPort.setPower(armPower);
            robot.armStar.setPower(armPower);
        }

    }
}
