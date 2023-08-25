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
import org.firstinspires.ftc.teamcode.NewRoadRunnerTest.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.NewRoadRunnerTest.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.NewRoadRunnerTest.trajectorysequence.TrajectorySequence;

@Config
@Autonomous (group = "Auto Testing")
public class RedCornerLow extends Threemaru2AutoBase {
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        super.runOpMode();
        initAuto();
        resetArm(); resetDrive();
        robot.servoHand1.setPosition(CLOSED1.getPosition());
        robot.servoHand2.setPosition(CLOSED2.getPosition());
        robot.servoExtend.setPosition(RETRACTED.getPosition());
        robot.motorTurret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorTurret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(new Pose2d())
                .splineToConstantHeading(new Vector2d(20, 0), Math.toRadians(0))//6,0,0
                .build();

        TrajectorySequence park1 = drive.trajectorySequenceBuilder(traj1.end())
                .back(14)
                .turn(Math.toRadians(95))
                .forward(23)
                .turn(Math.toRadians(-98))
                .splineToLinearHeading(new Pose2d(30, 23), Math.toRadians(0))
                .build();

        TrajectorySequence park2 = drive.trajectorySequenceBuilder(traj1.end())
                .forward(18)
                .build();

        TrajectorySequence park3 = drive.trajectorySequenceBuilder(traj1.end())
                .back(14)
                .turn(Math.toRadians(-95))//-89
                .forward(23)//23
                .turn(Math.toRadians(98))//101
                .splineToLinearHeading(new Pose2d(30, -21), Math.toRadians(0))
                .build();

        waitForStart();
        if (isStopRequested()) return;

        closeHand();
        armToPosition(100);
        drive.followTrajectorySequence(traj1);
        distDriveStar(-1, 4);
        //turretTimeBasedReset();
        extensionToPosition(RETRACTED.getPosition());
        armToPosition(1300);
        sleep(1000);
        PIDTurret(2400, 2);
        double distStar = robot.distSensorStarB.getDistance(DistanceUnit.CM);
        extensionToDistStar(distStar);
        sleep(1000);
        openHand();
        sleep(1000);
        closeHand();
        PIDTurret(500, 2);
        sleep(1000);
        extensionToPosition(RETRACTED.getPosition());
        if(sideOfSleeve == 1){
            armToPosition(0);
            drive.followTrajectorySequence(park1);
        } else if (sideOfSleeve == 11){
            armToPosition(0);
            drive.followTrajectorySequence(park3);
        } else {
            armToPosition(300);
            drive.followTrajectorySequence(park2);
        }
    }
}