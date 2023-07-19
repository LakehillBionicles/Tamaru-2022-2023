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
public class RedCorner1 extends ThreemaruAutoBase {
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        super.runOpMode();
        initAuto();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(new Pose2d())
                .splineToConstantHeading(new Vector2d(6, 0), Math.toRadians(0))
                .turn(Math.toRadians(-89))
                .forward(23)
                .turn(Math.toRadians(101))
                .waitSeconds(2)
                .splineToLinearHeading(new Pose2d(46, -21), Math.toRadians(-3))
                .addSpatialMarker(new Vector2d(46, -21), () -> armToPosition(3000))
                .waitSeconds(1)
                .build();

        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj1.end())
                .forward(6)
                .build();

        waitForStart();

        if (isStopRequested()) return;
        armToPosition(100);
        drive.followTrajectorySequence(traj1);
        distDriveStar(-1, 10);
        turretTimeBasedReset();
        double starDist = robot.distSensorStar.getDistance(DistanceUnit.CM);
        extensionToDistStar(starDist);
        openHand();
        turretToPosition(1800);
        //drive.followTrajectorySequence(traj2);
    }
}
