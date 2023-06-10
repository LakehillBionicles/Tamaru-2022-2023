package org.firstinspires.ftc.teamcode.CoordinateBased;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

//import org.firstinspires.ftc.teamcode.Threemaru.ThreemaruRoadRunner.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.Threemaru.ThreemaruRoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Tamaru3.Auto3.AutoBase;

@Disabled
@Config
@Autonomous(name = "coordinateTrajectoryTesting", group = "TrajectoryAutos")
public class coordinateTrajectoryTesting extends AutoBase {

    public Pose2d currentPose = new Pose2d(0,0,0);

    @Override
    public void runOpMode(){
        super.runOpMode();
        robot.init(hardwareMap);
        //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        resetArm();
        resetDrive();
        robot.servoHand.setPosition(robot.handClosed);

        Pose2d startPose = new Pose2d(0, 0, 0);

        //TrajectorySequence testSequence = drive.trajectorySequenceBuilder(startPose)
               // .addDisplacementMarker(() -> {  }) //arm up to high pole and turret star
               // .build();

        waitForStart();

        if (isStopRequested()) return;

       // drive.followTrajectorySequence(testSequence);
    }
}