package org.firstinspires.ftc.teamcode.Threemaru2.ThreemaruAuto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.NewRoadRunnerTest.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.NewRoadRunnerTest.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Threemaru.Auto4.ThreemaruAutoBase;

@Config
@Autonomous (group = "Auto Testing")
public class TestTurret extends Threemaru2AutoBase {
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        super.runOpMode();
        initAuto();

        waitForStart();

        if (isStopRequested()) return;

        encoderTurret(1000, 3);
        telemetry.addData("encoder", robot.motorTurret.getCurrentPosition());
        telemetry.update();
    }
}
