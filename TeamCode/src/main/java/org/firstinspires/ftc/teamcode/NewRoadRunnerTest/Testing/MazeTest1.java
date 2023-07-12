package org.firstinspires.ftc.teamcode.NewRoadRunnerTest.Testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.NewRoadRunnerTest.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.NewRoadRunnerTest.trajectorysequence.TrajectorySequence;
@Disabled
@Autonomous
public class MazeTest1 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        /*Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .splineToSplineHeading(new Pose2d(40, 6, Math.toRadians(90)), Math.toRadians(0))
                .build();*/

        TrajectorySequence maze = drive.trajectorySequenceBuilder(new Pose2d())
                .splineToLinearHeading(new Pose2d(40, 0, Math.toRadians(0)), Math.toRadians(0))
                .turn(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(40, 26, Math.toRadians(180)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(0, 30, Math.toRadians(90)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(0, 40, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(40, 40, Math.toRadians(0)), Math.toRadians(0))
                .build();

        TrajectorySequence mazeReverse = drive.trajectorySequenceBuilder(maze.end())
                .splineToLinearHeading(new Pose2d(0, 40, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(0, 30, Math.toRadians(90)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(40, 26, Math.toRadians(180)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(40, 0, Math.toRadians(90)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(5, -10, Math.toRadians(0)), Math.toRadians(0))
                .build();


        drive.followTrajectorySequence(maze);
        drive.followTrajectorySequence(mazeReverse);

        sleep(2000);

        /*drive.followTrajectory(
                drive.trajectoryBuilder(traj1.end(), true)
                        .splineToLinearHeading(new Pose2d(0, 0, 0), Math.toRadians(180))
                        .build()
        );*/
    }
}
