package org.firstinspires.ftc.teamcode.Tamaru3.Auto3;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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

        resetArm();
        resetDrive();
        robot.servoHand.setPosition(robot.handClosed);
        robot.servoExtend.setPosition(1);

        //Pose2d currentPose = new Pose2d(0, 0, 0);

        TrajectorySequence ScorePreload = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() -> { robot.servoHand.setPosition(robot.handClosed); armToPosition(highPoleArmTarget); }) //arm up to high pole
                .addTemporalMarker(.5, () -> {turretToPosition(robot.turretStar); extensionToPosition(robot.extensionStar);})
                .forward(28)//drive to signal cone
                .addDisplacementMarker(() -> {color = senseColorsFront(); telemetry.addData("color", color); telemetry.update();})
                .forward(40) //drive get signal cone out of way
                .back(2)//6, 4
                .build();

        TrajectorySequence wait = drive.trajectorySequenceBuilder(ScorePreload.end())
                .waitSeconds(.5)
                .build();

        TrajectorySequence shorterWait = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(.5)
                .build();

        TrajectorySequence PickUp1A = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() -> turretToPosition(robot.turretForward))
                .back(2)
                .addDisplacementMarker(() -> { armToPosition(fiveConeArmTarget); extensionToPosition(1);}) //arm down and turret forward
                .back(7)
                .turn(Math.toRadians(94)) //turn to stack
                .build();

        TrajectorySequence PickUp1B = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(12.0/2, 0))
                .forward(12)
                .build();

        TrajectorySequence PickUp1C = drive.trajectorySequenceBuilder(startPose)
                .forward(18)
                .build();

        TrajectorySequence Score1A = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() -> armToPosition(midPoleArmTarget)) //arm to low pole
                .lineTo(new Vector2d(-18, 0))//-37.5/2, -20
                .waitSeconds(.5)
                .build();

        TrajectorySequence Score1B = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() -> armToPosition(midPoleArmTarget)) //arm to low pole
                .addTemporalMarker((1), () -> { turretToPosition(robot.turretPort);})
                .lineTo(new Vector2d(-18.5, 0))//-36.5/2, -20
                .build();

        TrajectorySequence PickUp2 = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() -> { turretToPosition(robot.turretForward); extensionToPosition(1);})
                .addTemporalMarker((.5), () -> armToPosition(fourConeArmTarget))
                .lineTo(new Vector2d(21, 10))//21, 6
                .forward(7)//22
                .build();

        TrajectorySequence PickUp2B = drive.trajectorySequenceBuilder(startPose)
                .forward(15)
                .build();

        TrajectorySequence Score2 = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() -> armToPosition(lowPoleArmTarget)) //arm to low pole
                .waitSeconds(.5)
                .addTemporalMarker((1), () -> { turretToPosition(robot.turretPort);})
                .lineTo(new Vector2d(-10, 1))//-20.5
                .build();

        TrajectorySequence Score2B = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() -> armToPosition(lowPoleArmTarget)) //arm to low pole
                .addTemporalMarker((1), () -> { turretToPosition(robot.turretPort);})
                .lineTo(new Vector2d(1, 0))//-20.5
                .build();

        TrajectorySequence ParkGreen = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() -> {turretToPosition(robot.turretForward); extensionToPosition(1); robot.servoHand.setPosition(robot.handClosed);})
                .back(18)
                .back(18)
                .build();

        TrajectorySequence ParkBlue = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() -> {turretToPosition(robot.turretForward); extensionToPosition(1); robot.servoHand.setPosition(robot.handClosed);})
                .back(10)//14
                .build();

        TrajectorySequence ParkRed = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() -> {turretToPosition(robot.turretForward); extensionToPosition(1); })
                .forward(12)
                .build();

        waitForStart();

        if (isStopRequested()) return;
        ////////////SCORE PRELOADED CONE ON HIGH POLE///////////////////
        drive.followTrajectorySequence(ScorePreload);
        distDriveStar(-1);
        extensionToPosition(robot.extensionStar);
        //double distStar = robot.distSensorStar.getDistance(DistanceUnit.CM);
        //robot.servoExtend.setPosition(Math.min(1.45 + -0.145*distStar + 4.76E-04*distStar*distStar, .5));
        drive.followTrajectorySequence(wait);//for turret to settle
        robot.servoHand.setPosition(robot.handOpen);
        resetDrive();
        ////////////PICK UP A CONE FROM THE STACK///////////////////////
        drive.followTrajectorySequence(PickUp1A);
        resetDrive();
        drive.followTrajectorySequence(PickUp1B);
        lineUpWithConeStackPickUp();
        resetDrive();
        drive.followTrajectorySequence(PickUp1C);
        robot.servoHand.setPosition(robot.handClosed);
        correctAngle();
        drive.followTrajectorySequence(wait);
        ////////////SCORE ON MID POLE//////////////////////////////////
        resetDrive();
        drive.followTrajectorySequence(Score1A);
        lineUpWithConeStackScore();
        resetDrive();
        drive.followTrajectorySequence(Score1B);
        distDrivePort(-1);
        double distPort = (robot.distSensorPort.getDistance(DistanceUnit.CM)+robot.distSensorPort2.getDistance(DistanceUnit.CM))/2;
        //robot.servoExtend.setPosition(Math.min(1.33 + -0.188*distPort + 0.0104*distPort*distPort, .5));
        robot.servoExtend.setPosition(Math.max(1.12 + -0.103*distPort + 3.86E-03*distPort*distPort, .5));
        resetDrive();
        drive.followTrajectorySequence(wait);
        robot.servoHand.setPosition(robot.handOpen);
        ////////////PICK UP A CONE FROM THE STACK///////////////////////
        //color sensor find stack tape
        drive.followTrajectorySequence(PickUp2);
        lineUpWithConeStackPickUp();
        resetDrive();
        drive.followTrajectorySequence(PickUp2B);
        robot.servoHand.setPosition(robot.handClosed);
        correctAngle();
        drive.followTrajectorySequence(wait);
        ////////////SCORE ON LOW POLE//////////////////////////////////
        resetDrive();
        drive.followTrajectorySequence(Score2);
        lineUpWithConeStackScore();
        resetDrive();
        drive.followTrajectorySequence(Score2B);
        distDrivePort(-1);
        distPort = (robot.distSensorPort.getDistance(DistanceUnit.CM)+robot.distSensorPort2.getDistance(DistanceUnit.CM))/2;
        //robot.servoExtend.setPosition(Math.min(1.33 + -0.188*distPort + 0.0104*distPort*distPort, .5));
        robot.servoExtend.setPosition(Math.max(1.12 + -0.103*distPort + 3.86E-03*distPort*distPort, .6));
        drive.followTrajectorySequence(wait);
        robot.servoHand.setPosition(robot.handOpen);
        drive.followTrajectorySequence(wait);
        resetDrive();
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