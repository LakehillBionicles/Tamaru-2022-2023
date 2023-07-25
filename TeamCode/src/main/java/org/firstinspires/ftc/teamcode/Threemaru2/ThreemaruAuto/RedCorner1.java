package org.firstinspires.ftc.teamcode.Threemaru2.ThreemaruAuto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.NewRoadRunnerTest.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.NewRoadRunnerTest.trajectorysequence.TrajectorySequence;

@Config
@Autonomous (group = "Auto Testing")
public class RedCorner1 extends Threemaru2AutoBase {
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        super.runOpMode();
        initAuto();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(new Pose2d())
                .splineToConstantHeading(new Vector2d(6, 0), Math.toRadians(0))//6,0,0
                .turn(Math.toRadians(-89))//-89
                .forward(23)//23
                .turn(Math.toRadians(101))//101
                .waitSeconds(2)//2
                .splineToLinearHeading(new Pose2d(46, -21), Math.toRadians(-3))
                .addSpatialMarker(new Vector2d(46, -21), () -> armToPosition(3000))
                .waitSeconds(1)
                .build();

        waitForStart();
        if (isStopRequested()) return;

        armToPosition(100);
        drive.followTrajectorySequence(traj1);
        distDriveStar(-1, 10);
        turretTimeBasedReset();
        double distStar = robot.distSensorStar.getDistance(DistanceUnit.CM);
        extensionToDistStar(distStar);
        openHand();
        encoderTurret(-1800, 5);
        armToPosition(0);
    }
}