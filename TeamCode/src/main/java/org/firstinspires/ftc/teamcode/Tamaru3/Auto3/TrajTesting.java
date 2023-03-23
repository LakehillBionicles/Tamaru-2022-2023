package org.firstinspires.ftc.teamcode.Tamaru3.Auto3;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name = "TrajTesting", group = "TrajectoryAutos")
public class TrajTesting extends AutoBase{

    private String color = "";

    @Override
    public void runOpMode(){
        super.runOpMode();
        robot.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, 0);

        drive.setPoseEstimate(startPose);

        resetArm();
        resetDrive();
        robot.servoHand.setPosition(robot.handClosed);
        robot.servoExtend.setPosition(0);

        //Pose2d currentPose = new Pose2d(0, 0, 0);

        TrajectorySequence ScorePreload = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() -> { robot.servoHand.setPosition(robot.handClosed); armToPosition(highPoleArmTarget); }) //arm up to high pole
                .addTemporalMarker(.5, () -> {turretToPosition(robot.turretStar); extensionToPosition(robot.extensionStar);})
                .forward(28)//drive to signal cone
                .addDisplacementMarker(() -> {color = senseColorsFront(); telemetry.addData("color", color); telemetry.update();})
                .forward(40) //drive get signal cone out of way
                .back(7)//10
                .build();

        TrajectorySequence PickUp1 = drive.trajectorySequenceBuilder(ScorePreload.end())
                .back(10)//6
                .addDisplacementMarker(() -> { armToPosition(fiveConeArmTarget); turretToPosition(robot.turretForward); extensionToPosition(0);}) //arm down and turret forward
                .turn(Math.toRadians(94)) //turn to stack
                .forward(12) //forward to stack 1
                .waitSeconds(.5)
                .forward(12)
                //.addDisplacementMarker(this::correctAngle)
                .build();

        TrajectorySequence wait = drive.trajectorySequenceBuilder(PickUp1.end())
                .waitSeconds(.75)
                .build();

        TrajectorySequence Score1 = drive.trajectorySequenceBuilder(PickUp1.end())
                .addDisplacementMarker(() -> armToPosition(midPoleArmTarget)) //arm to low pole
                .addTemporalMarker((1), () -> { turretToPosition(robot.turretPort); extensionToPosition(robot.extensionPort);})
                .back(22) //back to mid pole 1
                .strafeRight(4)//2
                .back(21)
                .build();

        TrajectorySequence PickUp2 = drive.trajectorySequenceBuilder(Score1.end())
                .addDisplacementMarker(() -> { turretToPosition(robot.turretForward); extensionToPosition(0);})
                .addTemporalMarker((.5), () -> armToPosition(fourConeArmTarget))
                .strafeLeft(3)//1+4
                .forward(44)
                .build();

        TrajectorySequence Score2 = drive.trajectorySequenceBuilder(PickUp2.end())
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
                .addTemporalMarker((.25), () -> armToPosition(0))
                .back(12)
                .build();

        TrajectorySequence ParkRed = drive.trajectorySequenceBuilder(Score2.end())
                .addDisplacementMarker(() -> {turretToPosition(robot.turretForward); extensionToPosition(0); })
                .addTemporalMarker((.25), () -> armToPosition(threeConeArmTarget))
                .forward(12)
                .build();


        waitForStart();

        if (isStopRequested()) return;

        while(opModeIsActive()) {
            ////////////SCORE PRELOADED CONE ON HIGH POLE///////////////////
            drive.followTrajectorySequence(ScorePreload);
            drive.followTrajectorySequence(wait);//for turret to settle
            robot.servoHand.setPosition(robot.handOpen);
            ////////////PICK UP A CONE FROM THE STACK///////////////////////
            //color sensor find stack tape
            drive.followTrajectorySequence(PickUp1);
            robot.servoHand.setPosition(robot.handClosed);
            correctAngle();
            drive.followTrajectorySequence(wait);
            //touch sensor angle correction
            ////////////SCORE ON MID POLE//////////////////////////////////
            drive.followTrajectorySequence(Score1);
            robot.servoHand.setPosition(robot.handOpen);
            drive.followTrajectorySequence(wait);
            ////////////PICK UP A CONE FROM THE STACK///////////////////////
            //color sensor find stack tape
            drive.followTrajectorySequence(PickUp2);
            robot.servoHand.setPosition(robot.handClosed);
            correctAngle();
            drive.followTrajectorySequence(wait);
            ////////////SCORE ON LOW POLE//////////////////////////////////
            drive.followTrajectorySequence(Score2);
            robot.servoHand.setPosition(robot.handOpen);
            drive.followTrajectorySequence(wait);
            ////////////PARK//////////////////////////////////////////////
            if (color.equals("blue")) {
                drive.followTrajectorySequence(ParkBlue);
            } else if (color.equals("green")) {
                drive.followTrajectorySequence(ParkGreen);
            } else {
                drive.followTrajectorySequence(ParkRed);
            }
        }
    }
}