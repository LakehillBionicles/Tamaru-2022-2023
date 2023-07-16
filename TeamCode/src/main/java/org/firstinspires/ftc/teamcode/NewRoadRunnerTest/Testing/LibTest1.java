package org.firstinspires.ftc.teamcode.NewRoadRunnerTest.Testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.NewRoadRunnerTest.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.NewRoadRunnerTest.trajectorysequence.TrajectorySequence;

@Autonomous
public class LibTest1 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(new Pose2d())
                .splineToConstantHeading(new Vector2d(60, 2), Math.toRadians(0))
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
