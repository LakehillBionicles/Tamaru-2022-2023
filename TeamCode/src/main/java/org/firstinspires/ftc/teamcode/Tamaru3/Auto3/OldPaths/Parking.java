package org.firstinspires.ftc.teamcode.Tamaru3.Auto3.OldPaths;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Threemaru.ThreemaruRoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Tamaru3.Auto3.AutoBase;

@Disabled
@Config
@Autonomous(name = "Parking", group = "TrajectoryAutos")
public class Parking extends AutoBase {

    @Override
    public void runOpMode(){
        super.runOpMode();
        robot.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        resetArm();

        Trajectory moveToSignalCone = drive.trajectoryBuilder(new Pose2d())
                .addDisplacementMarker(() -> {
                    armToPosition(fourConeArmTarget);
                })
                .forward(28)
                .build();

        Trajectory parkRed = drive.trajectoryBuilder(moveToSignalCone.end())
                .strafeLeft(28)
                .build();

        Trajectory parkBlue = drive.trajectoryBuilder(moveToSignalCone.end())
                .forward(8)
                .build();

        Trajectory parkGreen = drive.trajectoryBuilder(moveToSignalCone.end())
                .strafeRight(28)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(moveToSignalCone);
        if(senseColorsFront().equals("blue")){
            drive.followTrajectory(parkBlue);
        } else if(senseColorsFront().equals("green")){
            drive.followTrajectory(parkGreen);
        } else {
            drive.followTrajectory(parkRed);
        }
    }

}