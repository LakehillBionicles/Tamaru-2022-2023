package org.firstinspires.ftc.teamcode.Threemaru2.ThreemaruAuto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.NewRoadRunnerTest.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.NewRoadRunnerTest.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Threemaru.Auto4.ThreemaruAutoBase;
@Config
@Autonomous (group = "Auto Testing")
public class RedCorner1Parking extends Threemaru2AutoBase{
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        super.runOpMode();
        initAuto();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        TrajectorySequence one = drive.trajectorySequenceBuilder(new Pose2d())
                //.splineToLinearHeading(new Pose2d(6, 0), Math.toRadians(0))//6,0,0
                .forward(6)
                .turn(Math.toRadians(94))
                .waitSeconds(2)
                //.forward(23)
                //.turn(Math.toRadians(-98))
                //.waitSeconds(2)
                //.splineToLinearHeading(new Pose2d(30, 21), Math.toRadians(0))
                .build();
        TrajectorySequence two = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(34)
                .build();
        TrajectorySequence three = drive.trajectorySequenceBuilder(new Pose2d())
                .splineToConstantHeading(new Vector2d(6, 0), Math.toRadians(0))//6,0,0
                .turn(Math.toRadians(-89))//-89
                .forward(23)//23
                .turn(Math.toRadians(101))//101
                .waitSeconds(2)//2
                .splineToLinearHeading(new Pose2d(46, -21), Math.toRadians(-3))
                .build();
        waitForStart();

        if (isStopRequested()) return;
        armToPosition(100);
        if(sideOfSleeve == 1) {
            drive.followTrajectorySequence(one);
        } else if(sideOfSleeve == 3) {
            drive.followTrajectorySequence(three);
        } else {
            drive.followTrajectorySequence(two);
        }
        //drive.followTrajectorySequence(traj2);
    }
}
