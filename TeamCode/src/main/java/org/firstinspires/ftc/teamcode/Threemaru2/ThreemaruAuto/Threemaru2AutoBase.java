package org.firstinspires.ftc.teamcode.Threemaru2.ThreemaruAuto;

import static org.firstinspires.ftc.teamcode.Threemaru.Subsystems.HandSubsystem.HandPos.*;
import static org.firstinspires.ftc.teamcode.Threemaru.Subsystems.ExtensionSubsystem.ExtendPos.*;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Threemaru.ThreemaruVision.ConeDetection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Threemaru.ThreemaruHardware;
import org.firstinspires.ftc.teamcode.Threemaru.ThreemaruVision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.Threemaru2.Threemaru2Hardware;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


import java.util.ArrayList;

public class Threemaru2AutoBase extends LinearOpMode {
    public Threemaru2Hardware robot = new Threemaru2Hardware();

    public final int downArmTarget = 0, lowPoleArmTarget = 1200, midPoleArmTarget = 2000, highPoleArmTarget = 2800;
    public final int fiveConeArmTarget = 450, fourConeArmTarget = 350, threeConeArmTarget = 250, twoConeArmTarget = 150;

    private PIDController driveController, thetaController, turretController, armController;

    //public static double pY = 0.0275, iY = 0.00055, dY = 0;
    public static double pY = 0.0275, iY = 0.00055, dY = 0;
    public static double pTheta = 0.0075, iTheta = 0, dTheta = 0.0012;
    public static double pTurret = 0.0005, iTurret = 0.00001, dTurret = 0.0001;
    public static double pArm = 0.001, iArm = 0, dArm = 0.0001, gArm = 0.001;
    public static double maxVelocity = 4000;
    private String webcamName = "Webcam 1";

    public BNO055IMU imu;
    public Orientation robotTheta;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    private ConeDetection ConeDetection;
    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!

    //c920 intrinsics

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    //c270 intrinsics
    /*
    double fx = 1078.03779;
    double fy = 1084.50988;
    double cx = 580.850545;
    double cy = 245.959325;

     */
    //It might also be this for c270
    // UNITS ARE METERS
    double tagsize = 0.166;

    int numFramesWithoutDetection = 0;

    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;
    public int sideOfSleeve;


    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
        driveController = new PIDController(pY, iY, dY);
        thetaController = new PIDController(pTheta, iTheta, dTheta);
        turretController = new PIDController(pTurret, iTurret, dTurret);
        armController = new PIDController(pArm, iArm, dArm);


        //resetArm();
        //resetDrive();
        //scanSignalSleeve();
        //telemetryForVision();
        //resetCamera();
        //detectingCones();
        //waitForStart();
    }


    public void initAuto(){
        resetArm(); resetDrive();
        robot.servoHand1.setPosition(CLOSED1.getPosition());
        robot.servoHand2.setPosition(CLOSED2.getPosition());
        robot.servoExtend.setPosition(RETRACTED.getPosition());
        robot.motorTurret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorTurret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        scanSignalSleeve();
        telemetryForVision();
        resetCamera();
        detectingCones();
        //initIMU();
    }
    public void initIMU(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        byte AXIS_MAP_CONFIG_BYTE = 0x6;
        byte AXIS_MAP_SIGN_BYTE = 0x1;

        imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);
        sleep(100);
        imu.write8(BNO055IMU.Register.AXIS_MAP_CONFIG,AXIS_MAP_CONFIG_BYTE & 0x0F);
        imu.write8(BNO055IMU.Register.AXIS_MAP_SIGN,AXIS_MAP_SIGN_BYTE & 0x0F);
        imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.IMU.bVal & 0x0F);
        sleep(100);
    }
    public void resetCamera(){
        camera.stopStreaming();
        camera.closeCameraDeviceAsync(() -> {});
    }
    public void detectingCones() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        ConeDetection = new ConeDetection();
        camera.setPipeline(ConeDetection);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });}
    public void scanSignalSleeve(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
        boolean stayInLoop = true;
        while (!opModeIsActive()&& stayInLoop){
            telemetry.addData("isItInOpMode","yes");
            telemetry.update();
            // Calling getDetectionsUpdate() will only return an object if there was a new frame
            // processed since the last time we called it. Otherwise, it will return null. This
            // enables us to only run logic when there has been a new frame, as opposed to the
            // getLatestDetections() method which will always return an object.
            ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();
            // If there's been a new frame...
            if(detections != null)
            {
                telemetry.addData("FPS", camera.getFps());
                telemetry.addData("Overhead ms", camera.getOverheadTimeMs());
                telemetry.addData("Pipeline ms", camera.getPipelineTimeMs());

                // If we don't see any tags
                if(detections.size() == 0)
                {
                    numFramesWithoutDetection++;

                    // If we haven't seen a tag for a few frames, lower the decimation
                    // so we can hopefully pick one up if we're e.g. far back
                    if(numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION)
                    {
                        aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW);
                    }
                }
                // We do see tags!
                else
                {
                    numFramesWithoutDetection = 0;

                    // If the target is within 1 meter, turn on high decimation to
                    // increase the frame rate
                    if(detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS)
                    {
                        aprilTagDetectionPipeline.setDecimation(DECIMATION_HIGH);
                    }

                    for(AprilTagDetection detection : detections)
                    {
                        sideOfSleeve = detection.id;
                        telemetry.addLine(String.format("\nDetected tag ID=%d", sideOfSleeve));
                        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
                        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
                        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
                        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
                        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
                        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
                        stayInLoop = false;
                    }
                }

                telemetry.update();
            }


            sleep(20);
        }
    }
    public void telemetryForVision(){
        telemetry.addLine(String.format("\nDetected tag ID=%d", sideOfSleeve));
        telemetry.update();
    }
    public void ControlAll(double yTarget, double thetaTarget, double armTarget, double turretTarget, double timeout){
        double robotY = ((robot.fpd.getCurrentPosition()+robot.bpd.getCurrentPosition() +robot.fsd.getCurrentPosition()+robot.bsd.getCurrentPosition())/4.0) / ThreemaruHardware.COUNTS_PER_INCH;
        driveController.setPID(pY, iY, dY);
        driveController.setSetPoint(yTarget + robotY);

        robotTheta = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        thetaController.setPID(pTheta, iTheta, dTheta);
        thetaController.setSetPoint(thetaTarget);

        double robotArm = (robot.armPort.getCurrentPosition()+robot.armStar.getCurrentPosition())/2.0;
        armController.setPID(pArm, iArm, dArm);
        armController.setSetPoint(armTarget);

        double robotTurret = robot.motorTurret.getCurrentPosition();
        turretController.setPID(pTurret, iTurret, dTurret);
        turretController.setSetPoint(turretTarget);

        resetRuntime();
        while (((!driveController.atSetPoint()||!thetaController.atSetPoint()||!armController.atSetPoint()||!turretController.atSetPoint()) && (getRuntime() < timeout))) {
            robotY = ((robot.fpd.getCurrentPosition()+robot.bpd.getCurrentPosition()+robot.fsd.getCurrentPosition()+robot.bsd.getCurrentPosition())/4.0) / ThreemaruHardware.COUNTS_PER_INCH;
            robotTheta = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
            robotArm = (robot.armPort.getCurrentPosition()+robot.armStar.getCurrentPosition())/2.0;
            robotTurret = robot.motorTurret.getCurrentPosition();

            double pidY = driveController.calculate(robotY, driveController.getSetPoint());
            double pidTheta = thetaController.calculate(robotTheta.firstAngle, thetaController.getSetPoint());
            double pidArm = armController.calculate(robotArm, armController.getSetPoint());
            double pidTurret = turretController.calculate(robotTurret, turretController.getSetPoint());

            double velocityY = pidY * maxVelocity;
            double velocityTheta = pidTheta * maxVelocity;
            double velocityArm = pidArm * maxVelocity;
            double velocityTurret = pidTurret * maxVelocity;

            robot.fpd.setVelocity(velocityY - velocityTheta);
            robot.bpd.setVelocity(velocityY - velocityTheta);
            robot.fsd.setVelocity(velocityY + velocityTheta);
            robot.bsd.setVelocity(velocityY + velocityTheta);

            robot.armPort.setVelocity(velocityArm);
            robot.armStar.setVelocity(velocityArm);

            robot.motorTurret.setVelocity(velocityTurret);
        }
    }
    public void encoderDrive(double speed, double leftInches, double rightInches) {
        int leftTarget = (robot.fpd.getCurrentPosition() + robot.bpd.getCurrentPosition())/2 + (int) (leftInches * robot.COUNTS_PER_INCH);
        int rightTarget = (robot.fsd.getCurrentPosition() + robot.bsd.getCurrentPosition())/2 + (int) (rightInches * robot.COUNTS_PER_INCH);
        robot.fpd.setTargetPosition(leftTarget);

        robot.bpd.setTargetPosition(leftTarget);
        robot.fsd.setTargetPosition(rightTarget);
        robot.bsd.setTargetPosition(rightTarget);

        robot.fpd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.bpd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.fsd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.bsd.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.fpd.setPower(speed);
        robot.bpd.setPower(speed);
        robot.fsd.setPower(speed);
        robot.bsd.setPower(speed);

        while(robot.fpd.isBusy()&&robot.bpd.isBusy()&&robot.fsd.isBusy()&&robot.bsd.isBusy()){}

        sleep(250);
    }
    public void PIDDrive(double yTarget, double thetaTarget, double timeout) {
        double robotY = ((robot.fpd.getCurrentPosition()+robot.bpd.getCurrentPosition() +robot.fsd.getCurrentPosition()+robot.bsd.getCurrentPosition())/4.0) / ThreemaruHardware.COUNTS_PER_INCH;
        robotTheta = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        driveController.setPID(pY, iY, dY);
        //driveController.setSetPoint(yTarget + robotY);
        driveController.setSetPoint(yTarget);
        //driveController.setTolerance(.1);
        thetaController.setPID(pTheta, iTheta, dTheta);
        //thetaController.setSetPoint(thetaTarget + robotTheta.firstAngle);
        thetaController.setSetPoint(thetaTarget);
        //thetaController.setTolerance(.1);

        resetRuntime();
        while (((!driveController.atSetPoint() || !thetaController.atSetPoint()) && (getRuntime() < timeout))) {
            robotY = ((robot.fpd.getCurrentPosition()+robot.bpd.getCurrentPosition()+robot.fsd.getCurrentPosition()+robot.bsd.getCurrentPosition())/4.0) / ThreemaruHardware.COUNTS_PER_INCH;
            robotTheta = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);

            double pidY = driveController.calculate(robotY, driveController.getSetPoint());
            double pidTheta = thetaController.calculate(robotTheta.firstAngle, thetaController.getSetPoint());

            double velocityY = pidY * maxVelocity;
            double velocityTheta = pidTheta * maxVelocity;

            robot.fpd.setVelocity(velocityY - velocityTheta);
            robot.bpd.setVelocity(velocityY - velocityTheta);
            robot.fsd.setVelocity(velocityY + velocityTheta);
            robot.bsd.setVelocity(velocityY + velocityTheta);

            telemetry.addData("robotTheta", robotTheta.firstAngle);
            telemetry.addData("targetTheta", thetaTarget);
            telemetry.addData("robotY", robotY);
            telemetry.addData("targetY", yTarget);
            telemetry.update();
        }
    }
    public void resetArm(){
        robot.armPort.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armStar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.armPort.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.armStar.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void resetDrive(){
        robot.fpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.fsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.fpd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.bpd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.fsd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.bsd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void armToPosition(int position){
        robot.armPort.setTargetPosition(position);
        robot.armStar.setTargetPosition(position);

        robot.armPort.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armStar.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.armPort.setPower(1);
        robot.armStar.setPower(1);
    }
    public void PIDArm(double target, double timeout) {
        armController.setPID(pArm, iArm, dArm);

        armController.setSetPoint(target);

        armController.setTolerance(.1);

        double state = (robot.armPort.getCurrentPosition()+robot.armStar.getCurrentPosition())/2.0;

        resetRuntime();
        while (((!armController.atSetPoint()) && (getRuntime() < timeout))) {

            state = (robot.armPort.getCurrentPosition()+robot.armStar.getCurrentPosition())/2.0;
            double pid = armController.calculate(state, target) + gArm;
            double velocity = pid * maxVelocity;

            robot.armPort.setVelocity(velocity);
            robot.armStar.setVelocity(velocity);
        }
    }
    public void distDrivePort(int direction, double timeout){
        resetRuntime();
        while((robot.distSensorPort.getDistance(DistanceUnit.CM)>10)&&getRuntime()<timeout){
            robot.fpd.setPower(direction*.2);
            robot.bpd.setPower(direction*.2);
            robot.fsd.setPower(direction*.2);
            robot.bsd.setPower(direction*.2);

            if(robot.distSensorPort.getDistance(DistanceUnit.CM)<10){
                break;
            }

            telemetry.addData("distPort", robot.distSensorPort.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
        robot.fpd.setPower(0);
        robot.bpd.setPower(0);
        robot.fsd.setPower(0);
        robot.bsd.setPower(0);
    }
    public void strafeDist(DistanceSensor distSensor, double targetDist, int direction){
        resetRuntime();
        while((distSensor.getDistance(DistanceUnit.CM)>targetDist||distSensor.getDistance(DistanceUnit.CM)>targetDist)&&getRuntime()<2){
            robot.fpd.setPower(direction*.4);
            robot.bpd.setPower(-direction*.4);
            robot.fsd.setPower(-direction*.4);
            robot.bsd.setPower(direction*.4);

            if(distSensor.getDistance(DistanceUnit.CM)<targetDist||distSensor.getDistance(DistanceUnit.CM)<targetDist){
                break;
            }
        }
        robot.fpd.setPower(0);
        robot.bpd.setPower(0);
        robot.fsd.setPower(0);
        robot.bsd.setPower(0);
    }
    public void distDriveStar(int direction, double timeout){
        resetRuntime();
        ArrayList<Double> distanceMeasurmentsB = new ArrayList<Double>();
        distanceMeasurmentsB.add(0,0.0);
        distanceMeasurmentsB.add(1,1.0);
        distanceMeasurmentsB.add(2,2.0);
        ArrayList<Double> distanceMeasurmentsT = new ArrayList<Double>();
        distanceMeasurmentsT.add(0,0.0);
        distanceMeasurmentsT.add(1,1.0);
        distanceMeasurmentsT.add(2,2.0);
        distanceMeasurmentsT.set(0, robot.distSensorStarT.getDistance(DistanceUnit.CM));
        boolean goInLoop = true;
        while(((robot.distSensorStarB.getDistance(DistanceUnit.CM)>25) || (distanceMeasurmentsB.get(1)>distanceMeasurmentsB.get(0))&&(distanceMeasurmentsB.get(2)>distanceMeasurmentsB.get(1))&&(distanceMeasurmentsB.get(2)>distanceMeasurmentsB.get(0))
                || (robot.distSensorStarT.getDistance(DistanceUnit.CM)>25) || (distanceMeasurmentsT.get(1)>distanceMeasurmentsT.get(0))&&(distanceMeasurmentsB.get(2)>distanceMeasurmentsB.get(1))&&(distanceMeasurmentsB.get(2)>distanceMeasurmentsB.get(0)))
                && (getRuntime()<timeout)||goInLoop){
            goInLoop = false;
            distanceMeasurmentsB.set(0, distanceMeasurmentsB.get(1));
            distanceMeasurmentsB.set(1, distanceMeasurmentsB.get(2));
            distanceMeasurmentsB.set(2, robot.distSensorStarB.getDistance(DistanceUnit.CM));
            distanceMeasurmentsT.set(0, distanceMeasurmentsT.get(1));
            distanceMeasurmentsB.set(1, distanceMeasurmentsB.get(2));
            distanceMeasurmentsT.set(2, robot.distSensorStarT.getDistance(DistanceUnit.CM));
            robot.fpd.setPower(direction*.3);
            robot.bpd.setPower(direction*.3);
            robot.fsd.setPower(direction*.3);
            robot.bsd.setPower(direction*.3);
        }
        robot.fpd.setPower(0);
        robot.bpd.setPower(0);
        robot.fsd.setPower(0);
        robot.bsd.setPower(0);
    }
    public void PIDTurret(double target, double timeout) {
        turretController.setPID(pTurret, iTurret, dTurret);

        turretController.setSetPoint(target);

        turretController.setTolerance(1);

        double turretPos = robot.motorTurret.getCurrentPosition();

        resetRuntime();
        while (((!turretController.atSetPoint()) && (getRuntime() < timeout))) {
            turretPos = robot.motorTurret.getCurrentPosition();

            double pid = turretController.calculate(turretPos, turretController.getSetPoint());
            robot.motorTurret.setPower(pid);
        }
        robot.motorTurret.setPower(0);
    }
    public void turretToPosition(int turretPosition){
        robot.motorTurret.setTargetPosition(turretPosition);

        robot.motorTurret.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.motorTurret.setPower(.3);
    }
    public void encoderTurret(int target, double timeout){
        resetRuntime();
        double state = robot.motorTurret.getCurrentPosition();
        while((state < (target-20))&&(getRuntime() < timeout)){
            robot.motorTurret.setPower(.08);
            state = robot.motorTurret.getCurrentPosition();
            telemetry.addData("turret pos", robot.motorTurret.getCurrentPosition());
            telemetry.update();
        }
        while((state > (target+20))&&(getRuntime() < timeout)){
            robot.motorTurret.setPower(-.08);
            state = robot.motorTurret.getCurrentPosition();
            telemetry.addData("turret pos", robot.motorTurret.getCurrentPosition());
            telemetry.update();
        }
        robot.motorTurret.setPower(0);
        telemetry.addData("turret pos", robot.motorTurret.getCurrentPosition());
        telemetry.update();
    }
    public void turretTimeBasedReset(){
        resetRuntime();
        while(getRuntime()<1){
            robot.motorTurret.setPower(.3);
        }
        robot.motorTurret.setPower(0);
        //robot.motorTurret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.motorTurret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void turretTimeBased(double direction, double timeout){
        resetRuntime();
        while(getRuntime()<timeout){
            robot.motorTurret.setPower(direction * .1);
        }
        robot.motorTurret.setPower(0);
    }
    public void extensionToPosition(double extensionPosition) {
        robot.servoExtend.setPosition(extensionPosition);
    }
    public void extensionToDistPort(double distPort){
        double extendPosition = Math.max((0.735 + -0.0247 * distPort + 3.31E-04 *distPort *distPort) + .025, .29);
        robot.servoExtend.setPosition(extendPosition);
    }
    public void extensionToDistStar(double distStar){
        double extendPosition = Math.max((0.668 + -0.0195 * distStar + 2.54E-04 * distStar * distStar)+.0125, .29);
        robot.servoExtend.setPosition(extendPosition);
    }
    public void openHand(){
        robot.servoHand1.setPosition(OPEN1.getPosition());
        robot.servoHand2.setPosition(OPEN2.getPosition());
    }
    public void closeHand(){
        robot.servoHand1.setPosition(CLOSED1.getPosition());
        robot.servoHand2.setPosition(CLOSED2.getPosition());
    }
    public void correctHeading(double timeout){
        resetRuntime();
        while((!robot.touchSensorPort.isPressed()||!robot.touchSensorStar.isPressed())&&getRuntime()<timeout) {
            while (!robot.touchSensorPort.isPressed() && robot.touchSensorStar.isPressed()) {
                robot.fpd.setPower(-.25);
                robot.bpd.setPower(-.25);
                robot.fsd.setPower(.25);
                robot.bsd.setPower(.25);
            }
            while (!robot.touchSensorStar.isPressed() && robot.touchSensorPort.isPressed()) {
                robot.fpd.setPower(.25);
                robot.bpd.setPower(.25);
                robot.fsd.setPower(-.25);
                robot.bsd.setPower(-.25);
            }
            while (!robot.touchSensorPort.isPressed() && !robot.touchSensorStar.isPressed()) {
                robot.fpd.setPower(-.5);
                robot.bpd.setPower(-.5);
                robot.fsd.setPower(-.5);
                robot.bsd.setPower(-.5);
            }
        }
    }
}