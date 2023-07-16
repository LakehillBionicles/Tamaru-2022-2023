package org.firstinspires.ftc.teamcode.Tamaru3.Auto3.OldPaths;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.NewRoadRunnerTest.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Tamaru3.Auto3.AutoBase;

@Disabled
@Config
@Autonomous(name = "ScoreHighRedCorner", group = "TrajectoryAutos")
public class ScoreHighRedCorner extends AutoBase {

    @Override
    public void runOpMode(){
        super.runOpMode();
        robot.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        resetArm();
        resetDrive();

        Trajectory moveToSignalCone = drive.trajectoryBuilder(new Pose2d())
                .addDisplacementMarker(() -> {
                    armToPosition(highPoleArmTarget);
                    robot.servoTurret.setPosition(robot.turretStar);
                })
                .forward(64)
                .build();

        Trajectory backToStack = drive.trajectoryBuilder(moveToSignalCone.end())
                .addDisplacementMarker(() -> {
                    armToPosition(800);
                    robot.servoTurret.setPosition(robot.turretForward);
                })
                .back(9)
                .build();

        Trajectory forwardToStack = drive.trajectoryBuilder(backToStack.end().plus(new Pose2d(0, 0, Math.toRadians(90))), false)
                .addDisplacementMarker(() -> {
                    armToPosition(800);
                    robot.servoTurret.setPosition(robot.turretForward);
                })
                .forward(28)
                .build();

        Trajectory backToLowPole = drive.trajectoryBuilder(forwardToStack.end())
                .addDisplacementMarker(() -> {
                    armToPosition(lowPoleArmTarget);
                    robot.servoTurret.setPosition(robot.turretPort);
                })
                .back(16)
                .build();

        Trajectory forwardToStackFromPole = drive.trajectoryBuilder(backToLowPole.end())
                .addDisplacementMarker(() -> {
                    armToPosition(800);
                    robot.servoTurret.setPosition(robot.turretForward);
                })
                .forward(16)
                .build();

        Trajectory backToLowPoleFromStack = drive.trajectoryBuilder(forwardToStackFromPole.end())
                .addDisplacementMarker(() -> {
                    armToPosition(lowPoleArmTarget);
                    robot.servoTurret.setPosition(robot.turretPort);
                })
                .back(16)
                .build();

        Trajectory backToGreenParking = drive.trajectoryBuilder(backToLowPole.end())
                .back(40)
                .build();


        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(moveToSignalCone);
        drive.followTrajectory(backToStack);
        drive.followTrajectory(forwardToStack);
        drive.followTrajectory(backToLowPole);
        drive.followTrajectory(forwardToStackFromPole);
        drive.followTrajectory(backToLowPoleFromStack);
        drive.followTrajectory(forwardToStackFromPole);
        drive.followTrajectory(backToLowPoleFromStack);
        drive.followTrajectory(forwardToStackFromPole);
        drive.followTrajectory(backToLowPoleFromStack);
        drive.followTrajectory(forwardToStackFromPole);
        drive.followTrajectory(backToLowPoleFromStack);
        drive.followTrajectory(backToGreenParking);
    }

}
