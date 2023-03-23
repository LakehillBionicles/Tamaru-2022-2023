package org.firstinspires.ftc.teamcode.Tamaru1.Auto1;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Tamaru1.TemaruHardware;
//import org.openftc.easyopencv.OpenCvCamera;

import java.util.ArrayList;

public class AutoBase extends LinearOpMode {

    public TemaruHardware robot = new TemaruHardware();
    //BNO055IMU imu;
    //Orientation angles;


    static final double FEET_PER_METER = 3.28084;


    //////////////////////////////// TARGET VALUES ////////////////////////////////////////////////
    public double newPOWTarget = 0;
    public double newSPOWTarget = 0;
    public double newSOWTarget = 0;
    public double newBOWTarget = 0;

    public double newTargetTheta = 0;
    public double newTargetX = 0;
    public double newTargetY = 0;

    //////////////////////////////////// ODOMETRY WHEEL CALCULATIONS //////////////////////////////////////////////
    public final double COUNTS_PER_ODO_REV = 8192;
    public final double ODO_GEAR_REDUCTION = (1.0); // This is < 1.0 if geared UP
    public final double ODO_WHEEL_DIAMETER_INCHES = 2.0;  // For figuring circumference
    public final double ODO_COUNTS_PER_INCH = ((COUNTS_PER_ODO_REV * ODO_GEAR_REDUCTION) /
            (ODO_WHEEL_DIAMETER_INCHES * 3.1415));

    public final double odoWheelGap = 11.5;

    //////////////////////////////// LOCATIONS ////////////////////////////////////////////////
    public double POWlocation = 0;
    public double SPOWlocation = 0;
    public double SOWlocation = 0;
    public double BOWlocation = 0;

    public double robotTheta = (-(((POWlocation - SOWlocation) / ODO_COUNTS_PER_INCH) / odoWheelGap));
    public double robotX = (BOWlocation / ODO_COUNTS_PER_INCH);
    public double robotXThetaCorrect = ((BOWlocation / ODO_COUNTS_PER_INCH) - (5 * robotTheta));
    public double robotY = (((POWlocation + SOWlocation) / 2) /ODO_COUNTS_PER_INCH);


    //////////////////////////////// DO WE REALLY NEED THESE ANYMORE???? ////////////////////////////////////////////////
    public double strafeError = 0;
    public double driveError = 0;
    public double oldRobotTheta = 0;
    public double oldThetaError = 0;


    //////////////////////////////// POWERS ////////////////////////////////////////////////
    public double thetaPower = 0;
    public double xPower = 0;
    public double yPower = 0;

    //////////////////////////////// ERRORS ////////////////////////////////////////////////
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

    //////////////////////////////// MISC ////////////////////////////////////////////////
    public double odoTime = 0;

    public double denominator = Math.max(Math.abs(yPower) + Math.abs(xPower) + Math.abs(thetaPower), 1);
    public double denominator2 = 0;

    public int SOWPos = 0;
    public int POWPos = 0;
    public int BOWPos = 0;

    public ElapsedTime runtime = new ElapsedTime();


    //////////////////////////////////// WHEEL CALCULATIONS //////////////////////////////////////////////
    static final double COUNTS_PER_MOTOR_REV = 560;
    static final double DRIVE_GEAR_REDUCTION = (0.11111); // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.0;  // For figuring circumference
    static final double COUNTS_PER_INCH = 1.2 * 4 * ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415));




    static final String sleeveColor = "";

    private double spedAdjust = .15;
    private int boundBE = 5;

    public final String VUFORIA_KEY = "AZjeccj/////AAABmZ7TkGdQaE90s4Gyo3b9T6oMtsulwtj5kAdhfhIabefDBj9bL1HNlKjYyp+p20rz5XXI3XDI+LJhqiNDUymG5F9OnRzuEMCWrAiD+KapcXmFnFqQE/1KtAdlOTLURn2zaOPk9yYQQnRuk4mKoIMNFSHbvD5jCcAEb2Xd6fCeFPXfUqof2JWKSklygJqup0mgtWOPlxb+PdPgRuGeSzTyZtOCuyGzny5vUTnno/ShUCH2Am56oJUwzvNJS22oBn1dwsPiNIZBJK/EkHfDzJPkxDLMQGP0r2FMDheJRy+nU/xQ///p26LxrG6Gm3MT1Wal7tVigS1IJEB0B+eoqK+6LBlRvDf+CFCBj9nXY7eIy9I1";


    /*** {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.*/
    public VuforiaLocalizer vuforia;


    /*** {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.*/
    public TFObjectDetector tfod;


    @Override
    public void runOpMode() {
//We might put the next lines in runOpMode in Startup
        startUp();
    }


    public void startUp() {
        robot.init(hardwareMap);


        ////////////reset encoders right here - added to make PID work
        robot.BOW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.POW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.SOW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);

        setMotorDir();
        //setMotorDirStrafe();

        robot.fpd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.bpd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.fsd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.bsd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.POW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.BOW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.SOW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.fpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.fsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.fsd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.fpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bsd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //initVuforia();

        //initTfod();

        telemetry.addData("Vision is Ready", ")");
        telemetry.update();


    }

    public void autoWhereAmI(){
        POWlocation = robot.POW.getCurrentPosition();
        SOWlocation = robot.SOW.getCurrentPosition();
        BOWlocation = robot.BOW.getCurrentPosition();

        robotTheta = (-(((POWlocation - SOWlocation) / ODO_COUNTS_PER_INCH) / odoWheelGap));
        robotX = (BOWlocation / ODO_COUNTS_PER_INCH) - (5 * robotTheta);
        robotY = (((robot.POW.getCurrentPosition() + robot.SOW.getCurrentPosition()) / 2) /ODO_COUNTS_PER_INCH);

        xError = (newTargetX - robotX);
        yError = (newTargetY - robotY);
        thetaError = (newTargetTheta - robotTheta);

        telemetry.addData("robotTheta:", robotTheta);
        telemetry.addData("robotX:", robotX);
        telemetry.addData("robotY:", robotY);

        telemetry.addData("thetaError:", thetaError);
        telemetry.addData("xError:", xError);
        telemetry.addData("yError:", yError);

        telemetry.update();

    }

    public void coordinateDrive(double thetaTarget, double xTarget, double yTarget, double rotPower, double linXPower, double linYPower, double rotTol, double xTol, double yTol){
        POWlocation = robot.POW.getCurrentPosition();
        SOWlocation = robot.SOW.getCurrentPosition();
        BOWlocation = robot.BOW.getCurrentPosition();

        robotTheta = (-(((POWlocation - SOWlocation) / ODO_COUNTS_PER_INCH) / odoWheelGap));
        robotX = (BOWlocation / ODO_COUNTS_PER_INCH)- (5 * robotTheta);
        robotY = (((robot.POW.getCurrentPosition() + robot.SOW.getCurrentPosition()) / 2) /ODO_COUNTS_PER_INCH);

        newTargetTheta = Math.toRadians(thetaTarget) + robotTheta;
        newTargetX = ((xTarget) + robotX);
        newTargetY = ((yTarget) + robotY);

        xError = (newTargetX - robotX);
        yError = (newTargetY - robotY);
        thetaError = (newTargetTheta - robotTheta);

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

        telemetry.update();

        while (Math.abs(thetaError) > Math.toRadians(rotTol) || Math.abs(xError) > xTol || Math.abs(yError) > yTol){

            odoTime = getRuntime();

            autoWhereAmI();

            denominator2 = Math.max(Math.abs(yPower) + Math.abs(xPower) + Math.abs(thetaPower), 1);

            robot.fpd.setPower(((linYPower * (yPower)) + (linXPower * (xPower)) + (rotPower * thetaPower))/denominator);
            robot.bpd.setPower(((linYPower * (yPower)) - (linXPower * (xPower)) + (rotPower * thetaPower))/denominator);
            robot.fsd.setPower(((linYPower * (yPower)) - (linXPower * (xPower)) - (rotPower * thetaPower))/denominator);
            robot.bsd.setPower(((linYPower * (yPower)) + (linXPower * (xPower)) - (rotPower * thetaPower))/denominator);

            if (Math.abs(thetaError) > Math.toRadians(rotTol)){
                thetaDeriv = ((thetaError - lastThetaError) / odoTime);
                thetaIntegralSum = (thetaIntegralSum + (thetaError * odoTime));
                if (thetaIntegralSum > integralSumLimit){
                    thetaIntegralSum = integralSumLimit;
                }
                if (thetaIntegralSum < -integralSumLimit){
                    thetaIntegralSum = -integralSumLimit;
                }
                thetaPower = -((Kd * thetaDeriv) + (Ki * thetaIntegralSum) + (Kp * Math.signum(thetaError)));
            }
            else{
                thetaPower = 0;
            }

            if (Math.abs(xError) > xTol){
                xDeriv = ((xError - lastXError) / odoTime);
                xIntegralSum = (xIntegralSum + (xError * odoTime));
                if (xIntegralSum > integralSumLimit){
                    xIntegralSum = integralSumLimit;
                }
                if (xIntegralSum < -integralSumLimit){
                    xIntegralSum = -integralSumLimit;
                }
                xPower = ((Kd * xDeriv) + (Ki * xIntegralSum) + (Kp * Math.signum(xError)));
            }
            else{
                xPower = 0;
            }

            if (Math.abs(yError) > yTol){
                yDeriv = ((yError - lastYError) / odoTime);
                yIntegralSum = (yIntegralSum + (yError * odoTime));
                if (yIntegralSum > integralSumLimit){
                    yIntegralSum = integralSumLimit;
                }
                if (yIntegralSum < -integralSumLimit){
                    yIntegralSum = -integralSumLimit;
                }
                yPower = ((Kd * yDeriv) + (Ki * yIntegralSum) + (Kp * Math.signum(yError)));
            }
            else{
                yPower = 0;
            }

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

            telemetry.addData("runtime", getRuntime());

            telemetry.update();

            //motor use to be here
        }

        robot.fpd.setPower(0);
        robot.bpd.setPower(0);
        robot.fsd.setPower(0);
        robot.bsd.setPower(0);
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

    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {  //forward back and turn with encoder, always do 3 less

        robot.fpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.fsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int newLeftTarget = 0;
        int newRightTarget = 0;

        setMotorDir();

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            telemetry.addData("counts per inch", this.COUNTS_PER_INCH);
            telemetry.update();

            newLeftTarget = (int) ((leftInches) * this.COUNTS_PER_INCH);
            newRightTarget = (int) ((rightInches) * this.COUNTS_PER_INCH);

            telemetry.addData("counts to run", newLeftTarget);
            telemetry.update();

            robot.fpd.setTargetPosition(newLeftTarget);
            robot.fsd.setTargetPosition(newRightTarget);
            robot.bpd.setTargetPosition(newLeftTarget);
            robot.bsd.setTargetPosition(newRightTarget);

            robot.fpd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.fsd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bpd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bsd.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //give the motors a 2% error allowance
            int NT1 = (int) (newLeftTarget * 0.981); //fp
            int NT2 = (int) (newRightTarget * 0.981); //fp

            // reset the timeout time and start motion.
            runtime.reset();

            robot.fpd.setPower(-Math.abs(speed));
            robot.fsd.setPower(-Math.abs(speed));
            robot.bsd.setPower(Math.abs(speed));
            robot.bpd.setPower(Math.abs(speed));

            while (robot.fpd.isBusy() && robot.fsd.isBusy() && robot.bpd.isBusy() && robot.bsd.isBusy() &&
                    opModeIsActive() && (runtime.seconds() < timeoutS) &&
                    ((Math.abs(robot.fpd.getCurrentPosition()) < Math.abs(NT1)) ||
                            (Math.abs(robot.fsd.getCurrentPosition()) < Math.abs(NT2)) ||
                            (Math.abs(robot.bpd.getCurrentPosition()) < Math.abs(NT1)) ||
                            (Math.abs(robot.bsd.getCurrentPosition()) < Math.abs(NT2)))) {
                //empty loop body that keeps running leaving motors running until time is up
            }

            robot.fpd.setPower(0);
            robot.fsd.setPower(0);
            robot.bsd.setPower(0);
            robot.bpd.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.fpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.fsd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bsd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.fpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.fsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.bpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.bsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            sleep(40);
        }
    }

    public void setMotorDirTurn(){
        robot.fsd.setDirection(DcMotorSimple.Direction.REVERSE); //Positive is counterclockwise
        robot.bsd.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.fpd.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.bpd.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void turn(double speed, double portInches, double starInches, double timeoutS){//Positive is counterclockwise. 1 inch = 6 degrees
        int newPortTarget = 0;
        int newStarTarget = 0;
        setMotorDirTurn();

        if (opModeIsActive()) {


            // Determine new target position, and pass to motor controller
            newPortTarget = robot.fpd.getCurrentPosition() + (int) ((portInches * this.COUNTS_PER_INCH)/6.67);
            newStarTarget = robot.bsd.getCurrentPosition() + (int) ((starInches * this.COUNTS_PER_INCH)/6.67);

            robot.fpd.setTargetPosition(newPortTarget);
            robot.fsd.setTargetPosition(newStarTarget);
            robot.bpd.setTargetPosition(newPortTarget);
            robot.bsd.setTargetPosition(newStarTarget);


            int turnNT1 = (int) (newPortTarget * 0.981); //fs
            int turnNT2 = (int) (newStarTarget * 0.981); //bs       //I don't know what NT Does


            robot.fpd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.fsd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bpd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bsd.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            robot.fpd.setPower(Math.abs(speed));  //-
            robot.fsd.setPower(Math.abs(speed));
            robot.bsd.setPower(Math.abs(speed));  //-
            robot.bpd.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    // ( front_star_wheel.isBusy() && front_port_wheel.isBusy()))
                    ((Math.abs(robot.fsd.getCurrentPosition()) < Math.abs(turnNT1)) || (Math.abs(robot.bsd.getCurrentPosition()) < Math.abs(turnNT2)))) { // took this out for now || (Math.abs(robot.bsd.getCurrentPosition()) < Math.abs(NT2))

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newPortTarget, newStarTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.fpd.getCurrentPosition(),
                        robot.bsd.getCurrentPosition());

                telemetry.update();
            }

            // Stop all motion;
            robot.fpd.setPower(0);
            robot.fsd.setPower(0);
            robot.bsd.setPower(0);
            robot.bpd.setPower(0);

            robot.fpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.fsd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bsd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            robot.fpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.fsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.bpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.bsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            sleep(250);   // optional pause after each move
        }



    }

    public void sideways(double speed, double frontInches, double backInches, double timeoutS) { //positive is right
        int newFrontTarget = 0;
        int newBackTarget = 0;

        setMotorDirStrafe();

        // Ensure that the opmode is still active
        if (opModeIsActive()) {


            // Determine new target position, and pass to motor controller
            newFrontTarget = robot.fsd.getCurrentPosition() + (int) (frontInches * this.COUNTS_PER_INCH);
            newBackTarget = robot.bsd.getCurrentPosition() + (int) (backInches * this.COUNTS_PER_INCH);

            robot.fpd.setTargetPosition(newFrontTarget);
            robot.fsd.setTargetPosition(newFrontTarget);
            robot.bpd.setTargetPosition(newBackTarget);
            robot.bsd.setTargetPosition(newBackTarget);


            int NT1 = (int) (newFrontTarget * 0.981); //fs
            int NT2 = (int) (newBackTarget * 0.981); //bs


            robot.fpd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.fsd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bpd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bsd.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            robot.fpd.setPower(Math.abs(speed));  //-
            robot.fsd.setPower(Math.abs(speed));
            robot.bsd.setPower(Math.abs(speed));  //-
            robot.bpd.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    // ( front_star_wheel.isBusy() && front_port_wheel.isBusy()))
                    ((Math.abs(robot.fsd.getCurrentPosition()) < Math.abs(NT1)) || (Math.abs(robot.bsd.getCurrentPosition()) < Math.abs(NT2)))) { // took this out for now || (Math.abs(robot.bsd.getCurrentPosition()) < Math.abs(NT2))

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newFrontTarget, newBackTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.fsd.getCurrentPosition(),
                        robot.bsd.getCurrentPosition());

                telemetry.update();
            }

            // Stop all motion;
            robot.fpd.setPower(0);
            robot.fsd.setPower(0);
            robot.bsd.setPower(0);
            robot.bpd.setPower(0);

            robot.fpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.fsd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bsd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            robot.fpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.fsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.bpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.bsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            sleep(250);   // optional pause after each move
        }


    }

    public void correctSideways(double speed) {

        double sped = (speed - spedAdjust);

        if (robot.bsd.getCurrentPosition() > boundBE) { // too much forward side

            while (robot.bsd.getCurrentPosition() > boundBE) {
                //decrease front side motor speed for adjustments, at spEd
                robot.fpd.setPower(sped);
                robot.fsd.setPower(sped);

            }

            //put front side motor speed back where it should be, at spEEd
            robot.fsd.setPower(speed);
            robot.fpd.setPower(speed);

        } else if (robot.bsd.getCurrentPosition() < -boundBE) {   //too much back side

            while (robot.bsd.getCurrentPosition() < -boundBE) {
                //decrease back side motor speed for adjustments, at spEd
                robot.bsd.setPower(sped);
                robot.bpd.setPower(sped);

            }

            //put back side motor speed back where it should be, at spEEd
            robot.bsd.setPower(sped);
            robot.bpd.setPower(sped);

        }
    }

    public void driveUntilTouch(double speed) { //function that drives until both touch sensors are pushed
        if (opModeIsActive()) {

            runtime.reset();

            setMotorDir();

            robot.fpd.setPower(Math.abs(speed));
            robot.fsd.setPower(Math.abs(speed));
            robot.bsd.setPower(Math.abs(speed));
            robot.bpd.setPower(Math.abs(speed));

            while (!(robot.touchSensorStar.isPressed()) || !(robot.touchSensorPort.isPressed())) {
                //Do nothing letting motors run until distance sensor sees a cone
            }

            robot.fpd.setPower(0);
            robot.fsd.setPower(0);
            robot.bsd.setPower(0);
            robot.bpd.setPower(0);

            // Turn off RUN_TO_POSITION
            /*robot.fpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.fsd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bsd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(40);*/
        }
    }

    public void resetEncoderTouch (){
        if(robot.touchSensorStar.isPressed() && robot.touchSensorPort.isPressed()){
            robot.SOW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.POW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.BOW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        } else {}

    }

    public void armLift ( double speed, double inches, double timeoutS){ //right now this function only lifts to the highest pole

        inches = inches / (2*3.14*23.8) * 560;
        // inches = inches*((560*23.8)/25.4);
        robot.BOW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.SOW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.BOW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.SOW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int newTarget = 0;

        if (opModeIsActive()) {

            newTarget = (int) inches; //inches * this.COUNTS_PER_INCH?
            robot.BOW.setTargetPosition(newTarget);
            robot.SOW.setTargetPosition(newTarget);
            robot.BOW.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.SOW.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            int NT = (int) (newTarget * 0.981); //fp
            runtime.reset();
            robot.BOW.setPower(Math.abs(speed));
            robot.SOW.setPower(Math.abs(speed*0.9));//0.9 is encoder ticks of arm 1 over arm2
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) && robot.BOW.isBusy() &&
                    ((Math.abs(robot.BOW.getCurrentPosition()) < Math.abs(NT)))) {
            }
            robot.BOW.setPower(0.6);
            robot.SOW.setPower(0.6);
            robot.BOW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.SOW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

    }

    public void armLiftDistance (double position){ //right now this function only lifts to the highest pole
        robot.BOW.setPower(2 * -(Math.sin((robot.distSensorArm.getDistance(DistanceUnit.CM) + robot.distSensorHighArm.getDistance(DistanceUnit.CM) + robot.distSensorLowerArm.getDistance(DistanceUnit.CM) - position) * (3.1415 / 4) / 85)));
        robot.SOW.setPower(2 * -(Math.sin((robot.distSensorArm.getDistance(DistanceUnit.CM) + robot.distSensorHighArm.getDistance(DistanceUnit.CM) + robot.distSensorLowerArm.getDistance(DistanceUnit.CM) - position) * (3.1415 / 4) / 85)));
    }

    public void armRunAllTheTime(){
        if (opModeIsActive()) {
            robot.BOW.setPower(0.1);
            robot.SOW.setPower(0.1);
        }
    }

    public void fourBar (double position){
        robot.arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        double FourBarCurrentPos;
        double FourBarTarget;

        FourBarCurrentPos = robot.arm2.getCurrentPosition();
        FourBarTarget = 0.0;

        FourBarTarget = position;
        robot.arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int NT = (int) (FourBarTarget * 0.981);
        robot.arm2.setPower(1.0);
        robot.arm2.setPower(Math.sin((-113) - FourBarCurrentPos)* (3.1415 / 4));
        telemetry.addData("yay", "");
        telemetry.update();
    }

    public void handGrab(){
        if ((robot.distSensorHand.getDistance(DistanceUnit.CM) < 13) && (robot.colorSensorHand.equals("red") || robot.colorSensorHand.equals("blue"))){
            telemetry.addData("i see:", "cone");
            telemetry.update();
            robot.servoFinger.setPosition(0.45);
        }
    }

    public void handDrop () {
        robot.servoFinger.setPosition(0.0);
        //might need to add a sleep an add in a second point if it dosen't drop fast enough
    }

    public void setMotorDir () { //make sure correct - not 100% sure
        robot.fsd.setDirection(DcMotorSimple.Direction.REVERSE); //forward
        robot.bsd.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.fpd.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.bpd.setDirection(DcMotorSimple.Direction.FORWARD); //neg??

    }

    public void setMotorDirStrafe () {
        robot.fsd.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.bsd.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.fpd.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.bpd.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void senseColorTelemetry(){ //drive until see not green
        while (opModeIsActive()){
            telemetry.addData("redStar", robot.colorSensorFront.red());
            telemetry.addData("greenStar", robot.colorSensorFront.green());
            telemetry.addData("blueStar", robot.colorSensorFront.blue());

            telemetry.addData("redPort", robot.colorSensorPort.red());
            telemetry.addData("greenPort", robot.colorSensorPort.green());
            telemetry.addData("bluePort", robot.colorSensorPort.blue());

            telemetry.update();

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











    /*public void initVuforia() {

         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }*/

    /*public void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.setZoom(1, 16.0 / 9.0);
        tfod.activate();
    }*/


}

