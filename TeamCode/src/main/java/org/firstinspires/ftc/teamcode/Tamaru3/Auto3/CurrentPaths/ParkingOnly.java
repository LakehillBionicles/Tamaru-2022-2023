package org.firstinspires.ftc.teamcode.Tamaru3.Auto3.CurrentPaths;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Tamaru3.Auto3.AutoBase;
@Disabled
@Config
@Autonomous(name = "ParkingOnly", group = "JustPark")
public class ParkingOnly extends AutoBase {
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

        TrajectorySequence ScanCone = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() -> { robot.servoHand.setPosition(robot.handClosed); armToPosition(lowPoleArmTarget); }) //arm up to high pole
                .forward(28)
                .addDisplacementMarker(() -> color = senseColorsFront())
                .build();

        TrajectorySequence ParkGreen = drive.trajectorySequenceBuilder(ScanCone.end())
                .strafeRight(28)
                .build();

        TrajectorySequence ParkBlue = drive.trajectorySequenceBuilder(ScanCone.end())
                .forward(8)
                .build();

        TrajectorySequence ParkRed = drive.trajectorySequenceBuilder(ScanCone.end())
                .strafeLeft(28)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySequence(ScanCone);
        if (color.equals("blue")) {
            drive.followTrajectorySequence(ParkBlue);
        } else if (color.equals("green")) {
            drive.followTrajectorySequence(ParkGreen);
        } else {
            drive.followTrajectorySequence(ParkRed);
        }
    }
}