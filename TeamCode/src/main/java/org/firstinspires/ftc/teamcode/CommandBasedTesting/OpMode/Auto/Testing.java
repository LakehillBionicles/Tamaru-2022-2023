package org.firstinspires.ftc.teamcode.CommandBasedTesting.OpMode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.CommandBasedTesting.Commands.ArmCommands.ArmToPole.armToHighPoleStar;
import org.firstinspires.ftc.teamcode.CommandBasedTesting.Commands.ArmCommands.ArmToPole.armToLowPolePort;
import org.firstinspires.ftc.teamcode.CommandBasedTesting.Commands.ArmCommands.ArmToStack.ArmToFiveCones;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;

@Disabled
@Config
@Autonomous(group="CommandBased")
public class Testing extends AutoBaseOpMode {

    @Override
    public void initialize() {
        super.initialize();
        tamaruArm.resetArm();
        tamaruDrivetrain.resetDrive();
        tamaruHand.grab();
    }

    @Override
    public void run() {
        super.run();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, 0);

        TrajectorySequence testSequence = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() -> { schedule(new armToHighPoleStar(tamaruArm, tamaruTurret, tamaruExtension)); }) //arm up to high pole and turret star
                .forward(64) //drive to high pole
                .addDisplacementMarker(() -> { tamaruHand.release(); }) //open hand
                .waitSeconds(1) //wait for cone to fall
                .addDisplacementMarker(() -> { schedule(new ArmToFiveCones(tamaruArm, tamaruTurret, tamaruExtension)); }) //arm down and turret forward
                .back(9) //back to stack
                .turn(Math.toRadians(90)) //turn to stack
                .forward(28) //forward to stack 1
                .addDisplacementMarker(() -> { tamaruHand.grab(); }) //close hand
                .waitSeconds(1) //wait for cone to fall
                .addDisplacementMarker(() -> { schedule(new armToLowPolePort(tamaruArm, tamaruTurret, tamaruExtension)); }) //arm to low pole and turret port
                .back(16) //back to low pole 1
                .addDisplacementMarker(() -> { tamaruHand.release(); }) //open hand
                .waitSeconds(1) //wait for cone to fall
                .back(40) //back to park green
                .build();

        drive.followTrajectorySequence(testSequence);
    }
}