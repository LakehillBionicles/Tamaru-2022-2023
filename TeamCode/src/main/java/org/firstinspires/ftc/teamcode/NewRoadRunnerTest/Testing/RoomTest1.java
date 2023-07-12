package org.firstinspires.ftc.teamcode.NewRoadRunnerTest.Testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.NewRoadRunnerTest.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.NewRoadRunnerTest.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.NewRoadRunnerTest.trajectorysequence.TrajectorySequence;

@Autonomous
public class RoomTest1 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        /*Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .splineToSplineHeading(new Pose2d(40, 6, Math.toRadians(90)), Math.toRadians(0))
                .build();*/

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(new Pose2d())
                .splineToLinearHeading(new Pose2d(25, 0, Math.toRadians(0)), Math.toRadians(0))
                //.waitSeconds(.25)
                //.turn(Math.toRadians(92))
                .waitSeconds(.25)
                .splineToConstantHeading(new Vector2d(29, 27), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(10))
                .waitSeconds(.25)
                .turn(Math.toRadians(-93))
                .waitSeconds(.25)
                .splineToConstantHeading(new Vector2d(29+16, 22), Math.toRadians(93),
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(10))
                .waitSeconds(.25)
                .turn(Math.toRadians(-94))
                .waitSeconds(.25)
                .splineToConstantHeading(new Vector2d(29+10, 7.5), Math.toRadians(94),
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(10))
                .waitSeconds(.25)
                .turn(Math.toRadians(95))
                .waitSeconds(.25)
                .splineToConstantHeading(new Vector2d(29+35, 0), Math.toRadians(95),
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(10))
                .build();

        /*TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj1.end())
                //.waitSeconds(.25)
                //.splineToConstantHeading(new Vector2d(29+16, 10), Math.toRadians(94))
                .build();*/

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySequence(traj1);
        //drive.followTrajectorySequence(traj2);

    }
}
