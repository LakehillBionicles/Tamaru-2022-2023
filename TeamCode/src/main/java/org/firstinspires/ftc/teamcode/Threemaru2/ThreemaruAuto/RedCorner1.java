package org.firstinspires.ftc.teamcode.Threemaru2.ThreemaruAuto;

import static org.firstinspires.ftc.teamcode.Threemaru.Subsystems.ExtensionSubsystem.ExtendPos.EXTENDED;
import static org.firstinspires.ftc.teamcode.Threemaru.Subsystems.ExtensionSubsystem.ExtendPos.RETRACTED;
import static org.firstinspires.ftc.teamcode.Threemaru.Subsystems.HandSubsystem.HandPos.CLOSED1;
import static org.firstinspires.ftc.teamcode.Threemaru.Subsystems.HandSubsystem.HandPos.CLOSED2;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.NewRoadRunnerTest.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.NewRoadRunnerTest.trajectorysequence.TrajectorySequence;

@Config
@Autonomous (group = "Auto Testing")
public class RedCorner1 extends Threemaru2AutoBase {
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        super.runOpMode();
        //initAuto();
        resetArm(); resetDrive();
        robot.servoHand1.setPosition(CLOSED1.getPosition());
        robot.servoHand2.setPosition(CLOSED2.getPosition());
        robot.servoExtend.setPosition(RETRACTED.getPosition());
        robot.motorTurret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorTurret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(new Pose2d())
                //.forward(20)
                .splineToConstantHeading(new Vector2d(6, 0), Math.toRadians(0))//6,0,0
                .turn(Math.toRadians(-89))//-89
                .forward(23)//23
                .turn(Math.toRadians(101))//101
                .splineToLinearHeading(new Pose2d(46, -21), Math.toRadians(-3))
                .addSpatialMarker(new Vector2d(46, -21), () -> armToPosition(3000))
                .waitSeconds(1)
                .build();

        waitForStart();
        if (isStopRequested()) return;

        armToPosition(100);
        drive.followTrajectorySequence(traj1);
        distDriveStar(-1, 4);
        //turretTimeBasedReset();
        PIDTurret(2400, 2);
        double distStar = robot.distSensorStar.getDistance(DistanceUnit.CM);
        extensionToDistStar(distStar);
        //extensionToPosition(EXTENDED.getPosition());
        sleep(1000);
        openHand();
        sleep(1000);
        PIDTurret(500, 2);
        armToPosition(0);
        extensionToPosition(RETRACTED.getPosition());
    }
}