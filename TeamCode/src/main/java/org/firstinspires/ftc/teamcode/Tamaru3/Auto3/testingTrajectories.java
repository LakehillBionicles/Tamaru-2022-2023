package org.firstinspires.ftc.teamcode.Tamaru3.Auto3;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;

@Autonomous(name = "testingTrajectories", group = "autoTesting")
public class testingTrajectories extends LinearOpMode {
        @Override
        public void runOpMode() {
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

            Trajectory moveSignalCone = drive.trajectoryBuilder(new Pose2d())
                    .forward(20)
                    .forward(20)
                    .build();

            Trajectory moveSignalConeAgain = drive.trajectoryBuilder(moveSignalCone.end())
                    .forward(20)
                    .forward(20)
                    .build();

            Trajectory backToHighPole = drive.trajectoryBuilder(moveSignalConeAgain.end())
                    .back(24)
                    .build();

            waitForStart();

            if(isStopRequested()) return;

            drive.followTrajectory(moveSignalCone);
            drive.followTrajectory(moveSignalConeAgain);
            drive.followTrajectory(backToHighPole);
        }
}