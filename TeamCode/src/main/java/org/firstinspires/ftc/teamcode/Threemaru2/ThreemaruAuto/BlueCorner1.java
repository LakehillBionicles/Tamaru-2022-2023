package org.firstinspires.ftc.teamcode.Threemaru2.ThreemaruAuto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.NewRoadRunnerTest.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.NewRoadRunnerTest.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Threemaru.Auto4.ThreemaruAutoBase;

@Config
@Autonomous (group = "Auto Testing")
public class BlueCorner1 extends ThreemaruAutoBase {
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        super.runOpMode();
        initAuto();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(new Pose2d())
                .splineToConstantHeading(new Vector2d(6, 0), Math.toRadians(0))
                .turn(Math.toRadians(-89))
                .forward(23.5)
                .turn(Math.toRadians(105))
                .build();

        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj1.end())
                .splineToConstantHeading(new Vector2d(46, -15), Math.toRadians(15))
                .build();

        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(traj2.end())
                .forward(6)
                .build();

        waitForStart();

        if (isStopRequested()) return;
        drive.followTrajectorySequence(traj1);
        drive.followTrajectorySequence(traj2);
        armToPosition(3000);
        distDriveStar(-1, 10);
        //drive.followTrajectorySequence(traj3);
    }
}
