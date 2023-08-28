package org.firstinspires.ftc.teamcode.Threemaru2.ThreemaruAuto;

import static org.firstinspires.ftc.teamcode.Threemaru.Subsystems.ExtensionSubsystem.ExtendPos.EXTENDED;
import static org.firstinspires.ftc.teamcode.Threemaru.Subsystems.ExtensionSubsystem.ExtendPos.RETRACTED;
import static org.firstinspires.ftc.teamcode.Threemaru.Subsystems.HandSubsystem.HandPos.CLOSED1;
import static org.firstinspires.ftc.teamcode.Threemaru.Subsystems.HandSubsystem.HandPos.CLOSED2;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.NewRoadRunnerTest.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.NewRoadRunnerTest.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.NewRoadRunnerTest.trajectorysequence.TrajectorySequence;
@Disabled
@Config
@Autonomous (group = "Auto Testing")
public class RedCorner1 extends Threemaru2AutoBase {
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
                //.forward(20)
                .splineToConstantHeading(new Vector2d(6, 0), Math.toRadians(0))//6,0,0
                .turn(Math.toRadians(-89))//-89
                .forward(23)//23
                .turn(Math.toRadians(101))//101
                .splineToLinearHeading(new Pose2d(0, -21), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(10))
                .build();

        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj1.end())
                .splineToLinearHeading(new Pose2d(46, -22), Math.toRadians(-3), SampleMecanumDrive.getVelocityConstraint(42.5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(30))
                .addSpatialMarker(new Vector2d(46, -22), this::closeHand)
                .addSpatialMarker(new Vector2d(46, -22), () -> armToPosition(2900))
                .waitSeconds(1)
                .build();

        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(traj2.end())
                .forward(8, SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                SampleMecanumDrive.getAccelerationConstraint(10))
                //.turn(Math.toRadians(105))
                //.forward(20)
                /*.splineToConstantHeading((new Vector2d(56, 20)), Math.toRadians(-105)
                 */
                .build();

        TrajectorySequence park2A = drive.trajectorySequenceBuilder(traj3.end())
                .turn(Math.toRadians(105))
                .build();

        TrajectorySequence park2B = drive.trajectorySequenceBuilder(park2A.end())
                .splineToLinearHeading(new Pose2d(46, 0), Math.toRadians(105))
                .build();

        TrajectorySequence park1 = drive.trajectorySequenceBuilder(traj3.end())
                .turn(Math.toRadians(105))
                .forward(40)
                .build();

        waitForStart();
        if (isStopRequested()) return;

        closeHand();
        armToPosition(100);
        drive.followTrajectorySequence(traj1);
        correctHeading(4);
        drive.followTrajectorySequence(traj2);
        distDriveStar(-1, 4);
        //turretTimeBasedReset();
        extensionToPosition(RETRACTED.getPosition());
        PIDTurret(2400, 2);
        double distStar = robot.distSensorStarB.getDistance(DistanceUnit.CM);
        extensionToDistStar(distStar);
        telemetry.addData("distStar", distStar);
        telemetry.update();
        //extensionToPosition(EXTENDED.getPosition());
        sleep(1000);
        openHand();
        sleep(1000);
        PIDTurret(500, 2);
        armToPosition(500);
        sleep(1000);
        extensionToPosition(RETRACTED.getPosition());
        drive.followTrajectorySequence(traj3);
        if(sideOfSleeve == 1){
            drive.followTrajectorySequence(park1);
            armToPosition(0);
        } else if (sideOfSleeve == 2){
            drive.followTrajectorySequence(park2A);
            drive.followTrajectorySequence(park2B);
            armToPosition(0);
        } else {
            armToPosition(0);
        }
    }
}