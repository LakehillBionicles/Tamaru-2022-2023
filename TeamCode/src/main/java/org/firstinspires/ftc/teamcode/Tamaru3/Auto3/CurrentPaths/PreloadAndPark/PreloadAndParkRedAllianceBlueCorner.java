package org.firstinspires.ftc.teamcode.Tamaru3.Auto3.CurrentPaths.PreloadAndPark;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Tamaru3.Auto3.AutoBase;
@Config
@Autonomous(name = "PreloadAndParkRedAllianceBlueCorner", group = "PreloadAndPark")
public class PreloadAndParkRedAllianceBlueCorner extends AutoBase {
    private String color = "";

    @Override
    public void runOpMode() {
        super.runOpMode();
        robot.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, 0);

        resetArm();
        resetDrive();
        robot.servoHand.setPosition(robot.handClosed);
        robot.servoExtend.setPosition(1);
        lights("white");

        //Pose2d currentPose = new Pose2d(0, 0, 0);

        TrajectorySequence ScorePreload = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() -> { robot.servoHand.setPosition(robot.handClosed); armToPosition(highPoleArmTarget); }) //arm up to high pole
                .addTemporalMarker(.5, () -> {turretToPosition(robot.turretPort); extensionToPosition(robot.extensionPort);})
                .forward(28)//drive to signal cone
                .addDisplacementMarker(() -> {color = senseColorsFront();})
                .forward(40) //drive get signal cone out of way
                .back(2)//6, 4
                .build();

        TrajectorySequence wait = drive.trajectorySequenceBuilder(ScorePreload.end())
                .waitSeconds(.5)
                .build();

        TrajectorySequence shorterWait = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(.25)
                .build();

        TrajectorySequence PickUp1A = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() -> turretToPosition(robot.turretForward))
                .back(2)
                .addDisplacementMarker(() -> { turretToPosition(robot.turretForward); armToPosition(fiveConeArmTarget); extensionToPosition(1);}) //arm down and turret forward
                .back(7)
                .turn(Math.toRadians(-94)) //turn to stack
                .build();


        TrajectorySequence ParkGreenA = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() -> {turretToPosition(robot.turretForward); extensionToPosition(1); robot.servoHand.setPosition(robot.handClosed);})
                .back(18)
                //.back(20)
                .build();

        TrajectorySequence ParkGreenB = drive.trajectorySequenceBuilder(startPose)
                .forward(12)
                .build();

        TrajectorySequence ParkBlue = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() -> {turretToPosition(robot.turretForward); extensionToPosition(1); robot.servoHand.setPosition(robot.handClosed);})
                .back(14)//14
                .build();

        TrajectorySequence ParkRed = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() -> {turretToPosition(robot.turretForward); extensionToPosition(1); })
                .back(12)
                .build();

        waitForStart();

        if (isStopRequested()) return;
        ////////////SCORE PRELOADED CONE ON HIGH POLE///////////////////
        drive.followTrajectorySequence(ScorePreload);
        distDrivePort(-1, 2);
        extensionToPosition(robot.extensionPort);
        drive.followTrajectorySequence(shorterWait);//for turret to settle
        robot.servoHand.setPosition(robot.handOpen);
        resetDrive();
        ////////////PARK//////////////////////////////////////////////
        drive.followTrajectorySequence(PickUp1A);
        distDriveStar(1,8);
        strafeDist(robot.distSensorStar, 3, 1);
        resetDrive();
        //drive.followTrajectorySequence(PickUp1B);
        lineUpWithConeStackPickUpRight("red");
        resetDrive();
        if (color.equals("blue")) {
            robot.servoHand.setPosition(robot.handClosed);
            drive.followTrajectorySequence(ParkBlue);
        } else if (color.equals("green")) {
            robot.servoHand.setPosition(robot.handClosed);
            drive.followTrajectorySequence(ParkGreenA);
            distDrivePort(-1, 3);
            strafeDist(robot.distSensorStar, 4, -1);
            resetDrive();
            drive.followTrajectorySequence(ParkGreenB);
        } else {
            robot.servoHand.setPosition(robot.handClosed);
            drive.followTrajectorySequence(ParkRed);
            correctAngle();
        }
    }
}
