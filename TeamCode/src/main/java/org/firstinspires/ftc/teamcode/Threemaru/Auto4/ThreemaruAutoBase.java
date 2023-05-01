package org.firstinspires.ftc.teamcode.Threemaru.Auto4;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.Threemaru.ThreemaruVision.ConeDetection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Threemaru.Tele4.ThreemaruHardware;
import org.firstinspires.ftc.teamcode.Threemaru.ThreemaruVision.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;


public class ThreemaruAutoBase extends LinearOpMode {
    public ThreemaruHardware robot = new ThreemaruHardware();

    public final int downArmTarget = 0, lowPoleArmTarget = 1200, midPoleArmTarget = 2000, highPoleArmTarget = 2800;
    public final int fiveConeArmTarget = 450, fourConeArmTarget = 350, threeConeArmTarget = 250, twoConeArmTarget = 150;

    private PIDController driveController;

    public static double py = 0.0275, iy = 0.00055, dy = 0;
    public static double maxVelocity = 4000;
    private String webcamName = "Webcam 1";

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
    int sideOfSleeve;
    

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        driveController = new PIDController(py, iy, dy);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        resetArm();
        resetDrive();
        scanSignalSleeve();
        telemetryForVision();
        detectingCones();
        waitForStart();
    }
    public void detectingCones(){
        //Resetting camera might fix null pointer errors
        camera.stopStreaming();
        camera.closeCameraDevice();
        camera.closeCameraDeviceAsync(() -> {});
        ConeDetection = new ConeDetection();
        camera.setPipeline(ConeDetection);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                camera.resumeViewport();
            }
            @Override
            public void onError(int errorCode) {
                                         }
        })
    ;}
    public void rotate90Left(){
        encoderDrive(0.5,-8,-8);
    }
    public void rotate180(){
        encoderDrive(0.5,-16,-16);
    }
    public void rotate90Right(){
        encoderDrive(0.5,8,8);
    }
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
        int sideOfSleeve;
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

    public void PIDDrive(double target, double timeout) {
        driveController.setPID(py, iy, dy);

        driveController.setSetPoint(target);

        driveController.setTolerance(.1);

        double robotY = ((robot.fpd.getCurrentPosition()+robot.bpd.getCurrentPosition() +robot.fsd.getCurrentPosition()+robot.bsd.getCurrentPosition())/4.0) / ThreemaruHardware.COUNTS_PER_INCH;

        resetRuntime();
        while (((!driveController.atSetPoint()) && (getRuntime() < timeout))) {
            robotY = ((robot.fpd.getCurrentPosition()+robot.bpd.getCurrentPosition() +robot.fsd.getCurrentPosition()+robot.bsd.getCurrentPosition())/4.0) / ThreemaruHardware.COUNTS_PER_INCH;

            double pid = driveController.calculate(robotY, driveController.getSetPoint());
            double velocityY = pid * maxVelocity;

            robot.fpd.setVelocity(velocityY);
            robot.bpd.setVelocity(velocityY);
            robot.fsd.setVelocity(velocityY);
            robot.bsd.setVelocity(velocityY);
        }
    }
    public String senseColors(ColorSensor colorSensor) {
        String color = "blank";
        double redMax = colorSensor.red();
        int blueMax = colorSensor.blue();
        int greenMax = colorSensor.green();

        while (opModeIsActive() && color.equals("blank")) {
            if ((redMax > blueMax) && (redMax > greenMax)) {
                telemetry.addData("i see red", " ");
                telemetry.update();
                color = "red";
            } else if ((blueMax > redMax) && (blueMax > greenMax)) {
                telemetry.addData("i see blue", " ");
                telemetry.update();
                color = "blue";
            } else if ((greenMax > redMax) && (greenMax > blueMax)) {
                telemetry.addData("i see green", " ");
                telemetry.update();
                color = "green";
            } else {
                telemetry.addData("i see nothing", " ");
                telemetry.update();
                color = "no go";
            }

        }
        return color;
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
        while(robot.distSensorStar.getDistance(DistanceUnit.CM)>10 && getRuntime()<timeout){
            robot.fpd.setPower(direction*.2);
            robot.bpd.setPower(direction*.2);
            robot.fsd.setPower(direction*.2);
            robot.bsd.setPower(direction*.2);

            telemetry.addData("distStar1", robot.distSensorStar.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
        robot.fpd.setPower(0);
        robot.bpd.setPower(0);
        robot.fsd.setPower(0);
        robot.bsd.setPower(0);
    }

    public void turretToPosition(double turretPosition) {
        robot.servoTurret.setPosition(turretPosition);
    }

    public void extensionToPosition(double extensionPosition) {
        robot.servoExtend.setPosition(extensionPosition);
    }
}