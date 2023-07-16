package org.firstinspires.ftc.teamcode.Tamaru3.Auto3.OldPaths;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.NewRoadRunnerTest.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.NewRoadRunnerTest.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Tamaru3.Auto3.AutoBase;

@Disabled
@Config
@Autonomous(name = "RedCornerTesting", group = "TrajectoryAutos")
public class RedCornerTesting extends AutoBase {

    @Override
    public void runOpMode(){
        super.runOpMode();
        robot.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        resetArm();
        resetDrive();
        robot.servoHand.setPosition(robot.handClosed);

        Pose2d startPose = new Pose2d(0, 0, 0);

        TrajectorySequence testSequence = drive.trajectorySequenceBuilder(startPose)
                .strafeRight(23.5)
                .addDisplacementMarker(() -> { robot.servoHand.setPosition(robot.handClosed); armToPosition(highPoleArmTarget); }) //arm up to high pole
                .addTemporalMarker(.5, () -> {turretToPosition(robot.turretPort); extensionToPosition(0);})
                .forward(62) //drive to high pole
                .addDisplacementMarker(() -> { robot.servoHand.setPosition(robot.handOpen); }) //open hand
                .waitSeconds(1.5)
                .addDisplacementMarker(() -> { armToPosition(fiveConeArmTarget); turretToPosition(robot.turretForward); }) //arm down and turret forward
                .back(8) //back to stack
                .turn(Math.toRadians(90)) //turn to stack
                .forward(25) //forward to stack 1
                .addDisplacementMarker(() -> { robot.servoHand.setPosition(robot.handClosed); }) //close hand
                .waitSeconds(1)
                /*.addDisplacementMarker(() -> { armToPosition(lowPoleArmTarget); turretToPosition(robot.turretPort); }) //arm to low pole and turret port
                .back(16) //back to low pole 1
                .addDisplacementMarker(() -> { robot.servoHand.setPosition(robot.handOpen); })*/ //open hand
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySequence(testSequence);
    }
}