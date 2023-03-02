package org.firstinspires.ftc.teamcode.Tamaru2.TeleOp2.Old;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Tamaru2.BaseClasses.Tamaru2Hardware;


@TeleOp
@Disabled

//////////////////////gamepad1 is drive; gamepad 2 is arm/hand/pre-set distances//////////////////////

public class TeleOp2DriversBetter2 extends LinearOpMode {
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

    public final double odoWheelGap = 12.5; //used to be 11.5

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

    ///////////SWIVEL 90 VARIABLES////////////////
    private int i;
    private int j;
    private int k;
    private int swivelDirection;
    private double swivelTheta;
    private double swivelThetaTarget;
    private double swivelThetaError;
    private double swivelThetaTolerance;

    ///////////SWIVEL INCREMENTAL VARIABLES////////////////
    private int I;
    private int J;
    private int K;
    private int swivelIncrementalDirection;
    private double swivelIncrementalTheta;
    private double swivelIncrementalThetaTarget;
    private double swivelIncrementalThetaError;
    private double swivelIncrementalThetaTolerance;

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

        robot.fpd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.bpd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.fsd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.bsd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive()) {

            ///////////////////////////////////////////////////////////// GAMEPAD 1 //////////////////////////////////////////////////
            drivePower = gamepad1.left_stick_y;
            strafePower = -gamepad1.left_stick_x;
            rotatePower = -gamepad1.right_stick_x;

            ///////////////////////////////////////////////////////// MOTOR POWERS ////////////////////////////////////////////////////

            fpder = drivePower + strafePower + rotatePower;
            bpdPower = -drivePower + strafePower - rotatePower;
            fsdPower = drivePower - strafePower - rotatePower;
            bsdPower = -drivePower - strafePower + rotatePower;

            teleDenom = Math.max(Math.max(Math.abs(fpder), Math.abs(bpdPower)), Math.max(Math.abs(fsdPower), Math.abs(bsdPower)));

            robot.fpd.setPower(fpder / teleDenom);
            robot.fsd.setPower(fsdPower / teleDenom);
            robot.bpd.setPower(-(bpdPower / teleDenom));
            robot.bsd.setPower(-(bsdPower / teleDenom));

        }
    }
}
