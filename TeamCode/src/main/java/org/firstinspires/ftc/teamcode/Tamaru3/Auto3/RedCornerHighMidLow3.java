package org.firstinspires.ftc.teamcode.Tamaru3.Auto3;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name = "RedCornerHighMidLow3", group = "TrajectoryAutos")
public class RedCornerHighMidLow3 extends AutoBase{

    private String color = "";

    @Override
    public void runOpMode(){
        super.runOpMode();
        robot.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, 0);

        //drive.setPoseEstimate(startPose);

        resetArm();
        resetDrive();
        robot.servoHand.setPosition(robot.handClosed);
        robot.servoExtend.setPosition(1);

        //Pose2d currentPose = new Pose2d(0, 0, 0);

        TrajectorySequence ScorePreload = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() -> { robot.servoHand.setPosition(robot.handClosed); armToPosition(highPoleArmTarget); }) //arm up to high pole
                .addTemporalMarker(.5, () -> {turretToPosition(robot.turretStar); /*extensionToPosition(robot.extensionStar);*/})
                .forward(28)//drive to signal cone
                .addDisplacementMarker(() -> {color = senseColorsFront(); telemetry.addData("color", color); telemetry.update();})
                .forward(40) //drive get signal cone out of way
                .back(8)//7
                .build();

        TrajectorySequence wait = drive.trajectorySequenceBuilder(ScorePreload.end())
                .waitSeconds(.75)
                .build();

        TrajectorySequence PickUp1 = drive.trajectorySequenceBuilder(ScorePreload.end())
                .addDisplacementMarker(() -> turretToPosition(robot.turretForward))
                .back(2)
                .addDisplacementMarker(() -> { armToPosition(fiveConeArmTarget); extensionToPosition(1);}) //arm down and turret forward
                .back(7)
                .turn(Math.toRadians(94)) //turn to stack
                .build();


        waitForStart();

        if (isStopRequested()) return;
        ////////////SCORE PRELOADED CONE ON HIGH POLE///////////////////
        drive.followTrajectorySequence(ScorePreload);
        drive.followTrajectorySequence(wait);//for turret to settle
        robot.servoHand.setPosition(robot.handOpen);
        ////////////PICK UP A CONE FROM THE STACK///////////////////////
        drive.followTrajectorySequence(PickUp1);
        robot.servoHand.setPosition(robot.handClosed);
        correctAngle();
        drive.followTrajectorySequence(wait);
        ////////////SCORE ON MID POLE//////////////////////////////////
        robot.servoHand.setPosition(robot.handOpen);
        drive.followTrajectorySequence(wait);
        ////////////PICK UP A CONE FROM THE STACK///////////////////////
        robot.servoHand.setPosition(robot.handClosed);
        correctAngle();
        drive.followTrajectorySequence(wait);
        ////////////SCORE ON LOW POLE//////////////////////////////////
        robot.servoHand.setPosition(robot.handOpen);
        drive.followTrajectorySequence(wait);
        ////////////PARK//////////////////////////////////////////////
        /*if (color.equals("blue")) {
            drive.followTrajectorySequence(ParkBlue);
        } else if (color.equals("green")) {
            drive.followTrajectorySequence(ParkGreen);
        } else {
            drive.followTrajectorySequence(ParkRed);
        }*/
    }
}