package org.firstinspires.ftc.teamcode.Threemaru.Tamaru3.Auto3.OldPaths;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.NewRoadRunnerTest.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.NewRoadRunnerTest.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Threemaru.Tamaru3.Auto3.AutoBase;

@Disabled
@Config
@Autonomous(name = "TrajectorySequenceTest", group = "TrajectoryAutos")
public class TrajectorySequenceTest extends AutoBase {

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
                .forward(68) //drive get signal cone out of way
                .back(8)//14
                .build();

        TrajectorySequence PickUp1 = drive.trajectorySequenceBuilder(ScorePreload.end())
                .back(8)//2
                .addDisplacementMarker(() -> { armToPosition(fiveConeArmTarget); turretToPosition(robot.turretForward); }) //arm down and turret forward
                //.back(6) //back to stack
                .turn(Math.toRadians(90)) //turn to stack
                .forward(12) //forward to stack 1
                .strafeRight(6)
                .forward(12)
                .build();

        TrajectorySequence wait = drive.trajectorySequenceBuilder(PickUp1.end())
                .waitSeconds(.75)
                .build();

        TrajectorySequence Score1 = drive.trajectorySequenceBuilder(wait.end())
                .addDisplacementMarker(() -> { armToPosition(lowPoleArmTarget); }) //arm to low pole
                .addTemporalMarker((1), () -> { turretToPosition(robot.turretPort); extensionToPosition(robot.extensionPort);})
                .back(18) //back to low pole 1
                .strafeLeft(4)
                .build();

        TrajectorySequence PickUp2 = drive.trajectorySequenceBuilder(Score1.end())
                .addDisplacementMarker(() -> { turretToPosition(robot.turretForward); extensionToPosition(0);})
                .addTemporalMarker((.25), () -> { armToPosition(fourConeArmTarget); })
                .strafeRight(4)
                .waitSeconds(2)
                .forward(18)
                .build();

        TrajectorySequence ParkGreen = drive.trajectorySequenceBuilder(Score1.end())
                .addDisplacementMarker(() -> {turretToPosition(robot.turretForward); extensionToPosition(0); robot.servoHand.setPosition(robot.handClosed);})
                .addTemporalMarker((.25), () -> {armToPosition(0);})
                .back(36)

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
        drive.followTrajectorySequence(Score1);
        robot.servoHand.setPosition(robot.handOpen);
        drive.followTrajectorySequence(wait);
        drive.followTrajectorySequence(ParkGreen);
    }
}