package org.firstinspires.ftc.teamcode.Threemaru.Tamaru3.Auto3.OldPaths;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.NewRoadRunnerTest.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.NewRoadRunnerTest.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Threemaru.Tamaru3.Auto3.AutoBase;

@Disabled
@Config
@Autonomous(name = "RedCornerHighMidLow2", group = "TrajectoryAutos")
public class RedCornerHighMidLow2 extends AutoBase {

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
                .lineTo(new Vector2d(61, 7)) //forward 12 and right 8 //12, 61, 8
                //.forward(12) //forward to stack 1
                //.strafeRight(10)
                //.waitSeconds(.5)
                .forward(18)
                //.addDisplacementMarker(this::correctAngle)
                .build();


        TrajectorySequence Score1A = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() -> armToPosition(midPoleArmTarget)) //arm to low pole
                //.addTemporalMarker((1), () -> { turretToPosition(robot.turretPort); extensionToPosition(robot.extensionPort);})
                //.back(18) //back to mid pole 1
                //.strafeLeft(7)//2
                //.back(23.5)
                .lineTo(new Vector2d(-37.5/2, -20))//-24
                .waitSeconds(.5)
                .build();

        TrajectorySequence Score1B = drive.trajectorySequenceBuilder(Score1A.end())
                .addDisplacementMarker(() -> armToPosition(midPoleArmTarget)) //arm to low pole
                .addTemporalMarker((1), () -> { turretToPosition(robot.turretPort); extensionToPosition(robot.extensionPort);})
                .lineTo(new Vector2d(-38.5, -20))
                .build();

        TrajectorySequence PickUp2 = drive.trajectorySequenceBuilder(Score1B.end())
                .addDisplacementMarker(() -> { turretToPosition(robot.turretForward); extensionToPosition(1);})
                .addTemporalMarker((.5), () -> armToPosition(fourConeArmTarget))
                .lineTo(new Vector2d(-15.5, -23))//-22
                //.strafeRight(2)//1+4
                //.forward(22)
                //.strafeRight(3)
                .forward(22)
                .build();

        TrajectorySequence Score2 = drive.trajectorySequenceBuilder(PickUp2.end())
                .addDisplacementMarker(() -> armToPosition(lowPoleArmTarget)) //arm to low pole
                .addTemporalMarker((1), () -> { turretToPosition(robot.turretPort); extensionToPosition(robot.extensionPort);})
                /*.back(9) //back to low pole 1
                .strafeLeft(3)
                .back(9.5)*/
                .lineTo(new Vector2d(-14, -22))
                .build();

        TrajectorySequence ParkGreen = drive.trajectorySequenceBuilder(Score2.end())
                .addDisplacementMarker(() -> {turretToPosition(robot.turretForward); extensionToPosition(1); robot.servoHand.setPosition(robot.handClosed);})
                //.addTemporalMarker((.5), () -> armToPosition(threeConeArmTarget))
                .back(18)
                .back(18)
                .build();

        TrajectorySequence ParkBlue = drive.trajectorySequenceBuilder(Score2.end())
                .addDisplacementMarker(() -> {turretToPosition(robot.turretForward); extensionToPosition(1); robot.servoHand.setPosition(robot.handClosed);})
                //.addTemporalMarker((.5), () -> armToPosition(threeConeArmTarget))
                .back(14)
                //.strafeLeft(3)
                .build();

        TrajectorySequence ParkRed = drive.trajectorySequenceBuilder(Score2.end())
                .addDisplacementMarker(() -> {turretToPosition(robot.turretForward); extensionToPosition(1); })
                //.addTemporalMarker((.5), () -> armToPosition(threeConeArmTarget))
                .forward(12)
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
            resetDrive();
            drive.followTrajectorySequence(Score1A);
            drive.followTrajectorySequence(Score1B);
            drive.followTrajectorySequence(wait);
            robot.servoHand.setPosition(robot.handOpen);
            ////////////PICK UP A CONE FROM THE STACK///////////////////////
            //color sensor find stack tape
            drive.followTrajectorySequence(PickUp2);
            robot.servoHand.setPosition(robot.handClosed);
            correctAngle();
            drive.followTrajectorySequence(wait);
            ////////////SCORE ON LOW POLE//////////////////////////////////
            drive.followTrajectorySequence(Score2);
            drive.followTrajectorySequence(wait);
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