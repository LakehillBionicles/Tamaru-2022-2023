package org.firstinspires.ftc.teamcode.Tamaru1.TeleOp1;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Tamaru1.TemaruHardware;
import org.firstinspires.ftc.teamcode.Tamaru1.Auto1.AutoBase;


@TeleOp
@Disabled

//////////////////////gamepad1 is drive; gamepad 2 is arm/hand/pre-set distances//////////////////////

public class TeleOp2DriversBetterFieldCentric extends LinearOpMode {
    TemaruHardware robot = new TemaruHardware();

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

    private double fpdPower;
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

        telemetry.addData("update date", "____");
        telemetry.update();

        robot.arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.BOW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.SOW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        swivelThetaTarget = 0;
        i = 0;
        j=0;
        k=0;
        theta0 = 0;


        while (opModeIsActive()) {

            ///////////////////////////////////////////////////////////// GAMEPAD 1 //////////////////////////////////////////////////
            drivePower = gamepad1.left_stick_y;
            strafePower = -gamepad1.left_stick_x;
            rotatePower = -gamepad1.right_stick_x;

            if(gamepad1.dpad_up){
                robot.BOW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.SOW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.POW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                robot.BOW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.SOW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.POW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                theta0 = 0;
            }else if(gamepad1.dpad_down){
                robot.BOW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.SOW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.POW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                robot.BOW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.SOW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.POW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                theta0 = Math.PI;
            }else if(gamepad1.dpad_left){ //flipped left and right 1/27 7:10
                robot.BOW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.SOW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.POW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                robot.BOW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.SOW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.POW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                theta0 = (Math.PI)/2;
            }else if(gamepad1.dpad_right){
                robot.BOW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.SOW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.POW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                robot.BOW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.SOW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.POW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                theta0 = (3*Math.PI)/2;
            }

            if (gamepad1.left_bumper) {
                handPos = 0.5;
            } else if (gamepad1.right_bumper) {
                handPos = 1;
            }


            /////////////////////////////////////////////////////////// GAMEPAD 2 ////////////////////////////////////////////////////
            //armPower = -gamepad2.left_stick_y;

            if(gamepad2.left_stick_y < 0){
                armPower = -gamepad2.left_stick_y;
            } else if(gamepad2.left_stick_y > 0){
                armPower = -gamepad2.left_stick_y/8;
            } else{
                armPower = 0;
            }

            if(gamepad2.right_stick_y < 0){
                servoArmPower = 1;
            } else if(gamepad2.right_stick_y > 0){
                servoArmPower = -1;
            } else{
                servoArmPower = 0;
            }

            if (gamepad2.left_bumper) {
                handPos = 0.5;
            } else if (gamepad2.right_bumper) {
                handPos = 1;
            }

            /*if ((!(gamepad1.right_bumper) && !(gamepad2.right_bumper) && ((robot.distSensorHand.getDistance(DistanceUnit.CM) < 8)))){
                handPos = 0.5;
            }*/

            if (gamepad2.dpad_up) {
                robot.arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            ///////////////////SWIVEL 90//////////////////////////////
            swivelLocation = robot.arm2.getCurrentPosition();
            swivelTheta = ((Math.PI*2)*swivelLocation)/288;
            swivelThetaTarget = i*(Math.PI/2);//can flip direction of target
            swivelThetaError = -swivelTheta + swivelThetaTarget;

            if(gamepad2.a && i<2 && (Math.abs(swivelThetaError)<(Math.PI/32) || (j==0 && k==0))) {
                i=i+1;
                swivelDirection = -1;//can flip direction of motors
                j=1;
                k=0;
                swivelThetaTarget = i*(Math.PI/2);
                swivelThetaError = -swivelTheta + swivelThetaTarget;
            }
            if(gamepad2.b && i>-2 && (Math.abs(swivelThetaError)<(Math.PI/32) || (j==0 && k==0)) ){
                i=i-1;
                j=0;
                k=1;
                swivelDirection = 1;//can flip direction of motors
                swivelThetaTarget = i*(Math.PI/2);
                swivelThetaError = -swivelTheta + swivelThetaTarget;
            }

            if(Math.abs(swivelThetaError)<(Math.PI/32)){
                swivelPower = 0;
            } else {
                if (i == 0 && !(j==0 && k==0)) {
                    swivelPower = (k*swivelDirection * Math.sin(swivelThetaError))+(j*swivelDirection*-Math.sin(swivelThetaError));
                }
                if (i == 1 && !(j==0 && k==0)) {
                    swivelPower = (j*swivelDirection * -Math.sin(swivelThetaError))+(k*swivelDirection*Math.sin(swivelThetaError));
                }
                if (i == -1 && !(j==0 && k==0)) { //this works at least when B is pressed first
                    swivelPower = (k*swivelDirection * Math.sin(swivelThetaError))+(j*swivelDirection* -Math.sin(swivelThetaError));
                }
                if (i == 2 || i == -2 && !(j==0 && k==0)) {//switched sign on both
                    swivelPower = (k*swivelDirection * Math.sin(swivelThetaError))+(j*swivelDirection* -Math.sin(swivelThetaError));
                }
            }
            ////////////////////////////////////////////////////////////////////


            ///////////////////SWIVEL INCREMENTAL//////////////////////////////
            /*swivelLocation = robot.arm2.getCurrentPosition();
            swivelTheta = ((Math.PI*2)*swivelLocation)/288;
            swivelThetaTarget = (I*(Math.PI/2))/8;//can flip direction of target
            swivelThetaError = -swivelTheta + swivelThetaTarget;

            if(gamepad2.x && I<2 && (Math.abs(swivelThetaError)<(Math.PI/32) || (J==0 && K==0))) {
                I=I+1;
                swivelDirection = -1;//can flip direction of motors
                J=1;
                k=0;
                swivelThetaTarget = (I*(Math.PI/2))/8;//can flip direction of target
                swivelThetaError = -swivelTheta + swivelThetaTarget;
            }
            if(gamepad2.y && I>-2 && (Math.abs(swivelThetaError)<(Math.PI/32) || (J==0 && K==0)) ){
                I=I-1;
                J=0;
                K=1;
                swivelDirection = 1;//can flip direction of motors
                swivelThetaTarget = (I*(Math.PI/2))/8;//can flip direction of target
                swivelThetaError = -swivelTheta + swivelThetaTarget;
            }

            if(Math.abs(swivelThetaError)<(Math.PI/32)){
                swivelPower = 0;
            } else {
                if (I == 0 && !(J==0 && K==0)) {
                    swivelPower = (k*swivelDirection * Math.sin(swivelThetaError))+(j*swivelDirection*-Math.sin(swivelThetaError));
                }
                if (I == 1 && !(J==0 && K==0)) {
                    swivelPower = (j*swivelDirection * -Math.sin(swivelThetaError))+(k*swivelDirection*Math.sin(swivelThetaError));
                }
                if (I == -1 && !(J==0 && K==0)) { //this works at least when B is pressed first
                    swivelPower = (k*swivelDirection * Math.sin(swivelThetaError))+(j*swivelDirection* -Math.sin(swivelThetaError));
                }
                if (I == 2 || I == -2 && !(J==0 && K==0)) {//switched sign on both
                    swivelPower = (k*swivelDirection * Math.sin(swivelThetaError))+(j*swivelDirection* -Math.sin(swivelThetaError));
                }
            }
             */



            ////////////////////////////////////////////////////////// MATH //////////////////////////////////////////////////////////
            POWlocation = robot.POW.getCurrentPosition();
            SOWlocation = robot.SOW.getCurrentPosition();
            BOWlocation = robot.BOW.getCurrentPosition();

            thetaRobot = (-(((POWlocation - SOWlocation) / ODO_COUNTS_PER_INCH) / odoWheelGap));// we sure about this negative?

            thetaLeftJoystick = Math.atan2((-gamepad1.left_stick_x), (gamepad1.left_stick_y));
            //thetaRightJoystick = Math.atan2((-gamepad1.right_stick_x), (gamepad1.right_stick_y)); //check signs
            thetaFieldCentric = thetaRobot - thetaLeftJoystick + theta0; // -(Math.PI/2)

            thetaDiscrepency = thetaRightJoystick - thetaRobot + theta0 + Math.PI;
            //hypotenuseRight = Math.sqrt((gamepad1.right_stick_x * gamepad1.right_stick_x)+(gamepad1.right_stick_y * gamepad1.right_stick_y));
            //rotatePower = hypotenuseRight*(Math.sin(thetaDiscrepency / 2));

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

            robot.fpd.setPower((-1*(newY + newX + rotatePower))/drivePowerDenom);
            robot.bpd.setPower(((newY - newX + rotatePower))/drivePowerDenom);
            robot.fsd.setPower((-1*(newY - newX - rotatePower))/drivePowerDenom);
            robot.bsd.setPower(((newY + newX - rotatePower))/drivePowerDenom);

            robot.BOW.setPower(.8*(armPower));
            robot.SOW.setPower(.8*(-armPower));

            robot.arm2.setPower(swivelPower/10);

            robot.servoArm.setPower(servoArmPower);

            robot.servoFinger.setPosition(handPos);

            /*telemetry.addData("fpd", robot.fpd.getPower());
            telemetry.addData("bpd", robot.bpd.getPower());
            telemetry.addData("fsd", robot.fsd.getPower());
            telemetry.addData("bsd", robot.bsd.getPower());*/
            telemetry.addData("BOW", BOWlocation);
            telemetry.addData("SOW", SOWlocation);
            telemetry.addData("POW", POWlocation);
            /*telemetry.addData("theta0", theta0);
            telemetry.addData("thetaleftjoystick", thetaLeftJoystick);
            telemetry.addData("theta robot", thetaRobot);
            telemetry.addData("thetaFieldCentric", thetaFieldCentric);
            telemetry.addData("hypoleft", hypotenuseLeft);
            telemetry.addData("i", i);
            telemetry.addData("j", j);
            telemetry.addData("k", k);
            telemetry.addData("swivelThetaTarget", swivelThetaTarget);
            telemetry.addData("swivelThetaError", swivelThetaError);
            telemetry.addData("swivelTheta", swivelTheta);
            telemetry.addData("swivelPower", swivelPower);
            telemetry.addData("BOW Power", robot.BOW.getPower());
            telemetry.addData("SOW Power", robot.SOW.getPower());
             */
            telemetry.update();

        }
    }
}
