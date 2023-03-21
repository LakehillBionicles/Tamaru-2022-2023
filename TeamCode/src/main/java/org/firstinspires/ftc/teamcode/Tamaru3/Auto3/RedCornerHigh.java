package org.firstinspires.ftc.teamcode.Tamaru3.Auto3;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name = "RedCornerHigh", group = "TrajectoryAutos")
public class RedCornerHigh extends AutoBase{

    private String color = "";

    @Override
    public void runOpMode(){
        super.runOpMode();
        robot.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        resetArm();
        resetDrive();
        robot.servoHand.setPosition(robot.handClosed);

        Pose2d startPose = new Pose2d(0, 0, 0);

        TrajectorySequence ScorePreload = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() -> { robot.servoHand.setPosition(robot.handClosed); armToPosition(highPoleArmTarget); }) //arm up to high pole
                .addTemporalMarker(.5, () -> {turretToPosition(robot.turretStar); extensionToPosition(robot.extensionStar);})
                .forward(28)//drive to signal cone
                .addDisplacementMarker(() -> {color = scanCone(); telemetry.addData("color", color);})
                .forward(40) //drive get signal cone out of way
                .back(10)//14
                .build();

        TrajectorySequence PickUp1 = drive.trajectorySequenceBuilder(ScorePreload.end())
                .back(6)//2
                .addDisplacementMarker(() -> { armToPosition(fiveConeArmTarget); turretToPosition(robot.turretForward); }) //arm down and turret forward
                //.back(6) //back to stack
                .turn(Math.toRadians(90)) //turn to stack
                .forward(12) //forward to stack 1
                .strafeRight(1)
                .forward(12)
                .build();

        TrajectorySequence wait = drive.trajectorySequenceBuilder(PickUp1.end())
                .waitSeconds(.75)
                .build();

        TrajectorySequence Score1 = drive.trajectorySequenceBuilder(wait.end())
                .addDisplacementMarker(() -> armToPosition(midPoleArmTarget)) //arm to low pole
                .addTemporalMarker((1), () -> { turretToPosition(robot.turretPort); extensionToPosition(robot.extensionPort);})
                .back(43) //back to mid pole 1 //45
                //.strafeLeft(1)
                .build();

        TrajectorySequence PickUp2 = drive.trajectorySequenceBuilder(Score1.end())
                .addDisplacementMarker(() -> { turretToPosition(robot.turretForward); extensionToPosition(0);})
                .addTemporalMarker((.5), () -> armToPosition(fourConeArmTarget))
                .strafeRight(1)
                .waitSeconds(1)
                .forward(44)
                .build();

        TrajectorySequence Score2 = drive.trajectorySequenceBuilder(wait.end())
                .addDisplacementMarker(() -> armToPosition(lowPoleArmTarget)) //arm to low pole
                .addTemporalMarker((1), () -> { turretToPosition(robot.turretPort); extensionToPosition(.25);})
                .back(18) //back to low pole 1
                .build();

        TrajectorySequence ParkGreen = drive.trajectorySequenceBuilder(Score2.end())
                .addDisplacementMarker(() -> {turretToPosition(robot.turretForward); extensionToPosition(0); robot.servoHand.setPosition(robot.handClosed);})
                .addTemporalMarker((.25), () -> armToPosition(0))
                .back(18)
                .back(18)
                .build();

        TrajectorySequence ParkBlue = drive.trajectorySequenceBuilder(Score2.end())
                .addDisplacementMarker(() -> {turretToPosition(robot.turretForward); extensionToPosition(0); robot.servoHand.setPosition(robot.handClosed);})
                .addTemporalMarker((.25), () -> {armToPosition(0);})
                .back(12)
                .build();

        TrajectorySequence ParkRed = drive.trajectorySequenceBuilder(Score2.end())
                .addDisplacementMarker(() -> {turretToPosition(robot.turretForward); extensionToPosition(0); robot.servoHand.setPosition(robot.handClosed);})
                .addTemporalMarker((.25), () -> {armToPosition(0);})
                .forward(12)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySequence(ScorePreload);
        robot.servoHand.setPosition(robot.handOpen);
        drive.followTrajectorySequence(PickUp1);
        robot.servoHand.setPosition(robot.handClosed);
        drive.followTrajectorySequence(wait);
        drive.followTrajectorySequence(Score1);
        robot.servoHand.setPosition(robot.handOpen);
        drive.followTrajectorySequence(wait);
        drive.followTrajectorySequence(PickUp2);
        robot.servoHand.setPosition(robot.handClosed);
        drive.followTrajectorySequence(wait);
        drive.followTrajectorySequence(Score2);
        robot.servoHand.setPosition(robot.handOpen);
        drive.followTrajectorySequence(wait);
        if(color=="blue"){
            drive.followTrajectorySequence(ParkBlue);
        } else if(color=="green"){
            drive.followTrajectorySequence(ParkGreen);
        } else {
            drive.followTrajectorySequence(ParkRed);
        }
    }
}