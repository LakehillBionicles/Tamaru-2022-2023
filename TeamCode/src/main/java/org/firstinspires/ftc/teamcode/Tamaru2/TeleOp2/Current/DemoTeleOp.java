package org.firstinspires.ftc.teamcode.Tamaru2.TeleOp2.Current;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Tamaru2.BaseClasses.Tamaru2Hardware;


@TeleOp
@Disabled

//////////////////////gamepad1 is drive; gamepad 2 is arm/hand/pre-set distances//////////////////////

public class DemoTeleOp extends LinearOpMode {
    Tamaru2Hardware robot = new Tamaru2Hardware();

    //the below variables are private to keep everything within this class

    //gamepad 1 (drive) variable
    private double drivePower;
    private double strafePower;
    private double rotatePower;
    private int drivePowerDenom;

    private double hypotenuseLeft;
    private double hypotenuseRight;
    private double thetaFieldCentric;
    private double thetaRobot;
    private double thetaLeftJoystick;
    private double thetaRightJoystick;
    private double thetaDiscrepency;
    private double newX;
    private double newY;
    private double theta0;

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

    private double fpdPower;
    private double bpdPower;
    private double fsdPower;
    private double bsdPower;

    private double teleDenom;

    private double extendPower;
    private double turretPosition = robot.turretForward;


    public ElapsedTime runtime = new ElapsedTime();


    public void runOpMode() {

        robot.init(hardwareMap);

        waitForStart();

        telemetry.addData("update date", "____");
        telemetry.update();


        robot.armPort.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.armStar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        theta0 = 0;


        while (opModeIsActive()) {

            ///////////////////////////////////////////////////////////// GAMEPAD 1 //////////////////////////////////////////////////
            drivePower = gamepad1.left_stick_y;
            strafePower = -gamepad1.left_stick_x;
            rotatePower = -gamepad1.right_stick_x;

            if (gamepad1.dpad_up) {
                robot.fpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot. fsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.bpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                robot.fpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.fsd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.bpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                theta0 = 0;
            } else if (gamepad1.dpad_down) {
                robot.fpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.fsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.bpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                robot.fpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.fsd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.bpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                theta0 = Math.PI;
            } else if (gamepad1.dpad_left) { //flipped left and right 1/27 7:10
                robot.fpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.fsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.bpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                robot.fpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.fsd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.bpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                theta0 = (Math.PI) / 2;
            } else if (gamepad1.dpad_right) {
                robot.fpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.fsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.bpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                robot.fpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.fsd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.bpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                theta0 = (3 * Math.PI) / 2;
            }

            if (gamepad1.left_bumper) {
                handPos = 0.5;
            } else if (gamepad1.right_bumper) {
                handPos = 1;
            }


            /////////////////////////////////////////////////////////// GAMEPAD 2 ////////////////////////////////////////////////////
            //armPower = -gamepad2.left_stick_y;

            if (gamepad2.left_stick_y < 0) {
                armPower = -gamepad2.left_stick_y;
            } else if (gamepad2.left_stick_y > 0) {
                armPower = -gamepad2.left_stick_y;
            } else {
                armPower = 0;
            }

            if (gamepad2.left_trigger < 0) {
                extendPower = 1;
            } else if (gamepad2.right_trigger > 0) {
                extendPower = -1;
            } else {
                extendPower = 0;
            }

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

            ////////////////////////////////////////////////////////// MATH //////////////////////////////////////////////////////////
            POWlocation = robot.fpd.getCurrentPosition();
            SOWlocation = robot.fsd.getCurrentPosition();
            BOWlocation = robot.bpd.getCurrentPosition();

            thetaRobot = (-(((POWlocation - SOWlocation) / ODO_COUNTS_PER_INCH) / odoWheelGap));//-?

            thetaLeftJoystick = Math.atan2((-gamepad1.left_stick_x), (gamepad1.left_stick_y)); //-?
            thetaFieldCentric = thetaRobot - thetaLeftJoystick + theta0;

            hypotenuseLeft = Math.sqrt((gamepad1.left_stick_x * gamepad1.left_stick_x)+(gamepad1.left_stick_y * gamepad1.left_stick_y));
            newX = (hypotenuseLeft * Math.sin(thetaFieldCentric));
            newY =  -(hypotenuseLeft * Math.cos(thetaFieldCentric)); //-?


            ///////////////////////////////////////////////////////// MOTOR POWERS ////////////////////////////////////////////////////

            if(gamepad1.left_trigger > 0){
                drivePowerDenom = 3;
            } else if(gamepad1.right_trigger > 0){
                drivePowerDenom = 1;
            } else{
                drivePowerDenom = 2;
            }

            robot.fpd.setPower(((newY + newX + rotatePower))/drivePowerDenom);
            robot.bpd.setPower(((newY - newX + rotatePower))/drivePowerDenom);
            robot.fsd.setPower(((newY - newX - rotatePower))/drivePowerDenom);
            robot.bsd.setPower(((newY + newX - rotatePower))/drivePowerDenom);

            robot.armPort.setPower(armPower);
            robot.armStar.setPower(armPower);

            //robot.servoExtend.setPower(extendPower);

            robot.servoTurret.setPosition(turretPosition);

            robot.servoHand.setPosition(handPos);

        }
    }
}
