package org.firstinspires.ftc.teamcode.Threemaru.Auto4;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Threemaru.ThreemaruRoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Threemaru.ThreemaruRoadRunner.trajectorysequence.TrajectorySequence;

//@Disabled
@Config
@Autonomous(name = "TestAuto")
public class ThreemaruTestAuto extends ThreemaruAutoBase {

    @Override
    public void runOpMode() {
        super.runOpMode();
        robot.init(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        resetArm();
        resetDrive();
        scanSignalSleeve();
        telemetryForVision();
        robot.servoHand1.setPosition(.25);
        robot.servoHand2.setPosition(.4);
        robot.servoExtend.setPosition(.5);

        waitForStart();

        while(opModeIsActive()){
            PIDDrive(73, 5);
            /*distDriveStar(-1, 2);
            encoderDrive(.5, -12, -12);
            encoderDrive(.5, -9, 9);
            encoderDrive(.5, 10, 10);
            distDrivePort(1, 10);
            extensionToPosition(0);
            extensionToPosition(.5);
            if(sideOfSleeve == 1){
                encoderDrive(0.5,12,12);
            }else if(sideOfSleeve == 2){
                encoderDrive(.5, -6, -6);
            }else{
                encoderDrive(.5,-30,-30);
            }*/
        }
    }
}