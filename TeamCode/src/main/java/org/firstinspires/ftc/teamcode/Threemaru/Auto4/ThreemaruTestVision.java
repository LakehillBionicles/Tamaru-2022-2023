package org.firstinspires.ftc.teamcode.Threemaru.Auto4;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Threemaru.ThreemaruVision.ConeDetection;
import org.firstinspires.ftc.teamcode.Threemaru.ThreemaruVision.ConeDetectionRed;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Config
@Autonomous(name = "TestVision")
public class ThreemaruTestVision extends ThreemaruAutoBase {
    @Override
    public void runOpMode() {
        super.runOpMode();
        robot.init(hardwareMap);
        resetArm();
        resetDrive();
        telemetry.addData("beforeSignalSleeve", "yes");
        telemetry.update();
        scanSignalSleeve();
        telemetry.addData("AfterSignalSleeve", "yes?");
        telemetry.update();
        telemetryForVision();
        telemetry.addData("AfterTelemetry", "yes?");
        telemetry.update();
        //resetCamera();
        telemetry.addData("AfterResetCamera", "yes?");
        telemetry.update();
        //detectingCones();
        telemetry.addData("AfterDetectingCone", "yes?");
        telemetry.update();
        switchPipeline();
        camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        /*camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode) {
                    telemetry.addData("errorInsidenullLoop", ":(");
                }
        });

         */
        while(!opModeIsActive()){
            telemetry.addData("Before colors", "yes?");
            telemetry.addData("sideOfSleeve", sideOfSleeve);
            telemetry.addData("blueColor: ", ConeDetection.getBluePosition());
            telemetry.addData("RedColor: ", ConeDetection.getRedPosition());
            telemetry.update();
        }
        //waitForStart();
        if (opModeIsActive()) {
            //resetCamera();
            telemetry.addData("AfterresetCameraOpModeIf", "yes?");
            telemetry.update();
            //detectingCones();
            telemetry.addData("AfterdetectingConesOpModeIf", "yes?");
            telemetry.update();
            while (opModeIsActive()) {
                /*telemetry.addData("AfterWhile", "yes?");
                telemetry.update();
                //resetCamera();
                telemetry.addData("AfterresetCameraOpModeWhile", "yes?");
                telemetry.update();
                //detectingCones();
                 */
                telemetry.addData("Before colors", "yes?");
                telemetry.addData("sideOfSleeve", sideOfSleeve);
                telemetry.addData("blueColor: ", ConeDetection.getBluePosition());
                telemetry.addData("RedColor: ", ConeDetection.getRedPosition());
                telemetry.update();
                }
            }
        }
    }