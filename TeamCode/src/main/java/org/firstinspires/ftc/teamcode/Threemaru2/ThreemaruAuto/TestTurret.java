package org.firstinspires.ftc.teamcode.Threemaru2.ThreemaruAuto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.NewRoadRunnerTest.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.NewRoadRunnerTest.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Threemaru.Auto4.ThreemaruAutoBase;

@Disabled
@Config
@Autonomous (group = "Auto Testing")
public class TestTurret extends Threemaru2AutoBase {
    public static double target = -2000;
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        super.runOpMode();
        initAuto();
        //extensionToPosition(.29);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        if (isStopRequested()) return;

        //PIDTurret(target, 1000);
        //distDriveStar(-1, 10);
        while(robot.distSensorStarB.getDistance(DistanceUnit.CM)>50){
            robot.fpd.setPower(-.3);
            robot.bpd.setPower(-.3);
            robot.fsd.setPower(-.3);
            robot.bsd.setPower(-.3);

            telemetry.addData("distStar1", robot.distSensorStarB.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
        robot.fpd.setPower(0);
        robot.bpd.setPower(0);
        robot.fsd.setPower(0);
        robot.bsd.setPower(0);
        telemetry.addData("distStar1", robot.distSensorStarB.getDistance(DistanceUnit.CM));
        telemetry.update();

        telemetry.addData("encoder", robot.motorTurret.getCurrentPosition());
        telemetry.addData("target", target);
        telemetry.update();
    }
}