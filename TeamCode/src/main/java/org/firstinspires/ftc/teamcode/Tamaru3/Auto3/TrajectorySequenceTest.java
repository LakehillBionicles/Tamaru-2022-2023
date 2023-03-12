package org.firstinspires.ftc.teamcode.Tamaru3.Auto3;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name = "TrajectorySequenceTest", group = "TrajectoryAutos")
public class TrajectorySequenceTest extends AutoBase{

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
                .addDisplacementMarker(() -> { armToPosition(highPoleArmTarget); turretToPosition(robot.turretStar); }) //arm up to high pole and turret star
                .forward(64) //drive to high pole
                .addDisplacementMarker(() -> { robot.servoHand.setPosition(robot.handOpen); }) //open hand
                //.addDisplacementMarker(() -> { armToPosition(0); turretToPosition(robot.turretForward); }) //arm down and turret forward
                /*.back(9) //back to stack
                .turn(Math.toRadians(90)) //turn to stack
                .forward(28) //forward to stack 1
                .addDisplacementMarker(() -> { robot.servoHand.setPosition(robot.handClosed); }) //close hand
                .waitSeconds(1) //wait for cone to fall
                .addDisplacementMarker(() -> { armToPosition(lowPoleArmTarget); turretToPosition(robot.turretPort); }) //arm to low pole and turret port
                .back(16) //back to low pole 1
                .addDisplacementMarker(() -> { robot.servoHand.setPosition(robot.handOpen); })*/ //open hand
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySequence(testSequence);
    }
}