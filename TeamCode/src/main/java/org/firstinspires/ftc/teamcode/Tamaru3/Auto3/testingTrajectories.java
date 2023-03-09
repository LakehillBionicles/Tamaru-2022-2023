package org.firstinspires.ftc.teamcode.Tamaru3.Auto3;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Tamaru3.Tamaru3Hardware;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;

@Config
@Autonomous(name = "testingTrajectories", group = "autoTesting")
public class testingTrajectories extends LinearOpMode {
    Tamaru3Hardware robot = new Tamaru3Hardware();
    private PIDController armPID;
    public static double pArm = 0.01, iArm = 0.0001, dArm = 0.0002;

    public final int downArmTarget = 0, lowPoleArmTarget = 1400, midPoleArmTarget = 2000, highPoleArmTarget = 2600;
    public final int fiveConeArmTarget = 600, fourConeArmTarget = 500, threeConeArmTarget = 400, twoConeArmTarget = 300;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        armPID = new PIDController(pArm, iArm, dArm);

        Trajectory moveToSignalCone = drive.trajectoryBuilder(new Pose2d())
                .forward(28)
                .build();

        Trajectory parkRed = drive.trajectoryBuilder(moveToSignalCone.end())
                .strafeLeft(24)
                .build();

        Trajectory parkBlue = drive.trajectoryBuilder(moveToSignalCone.end())
                .forward(8)
                .build();

        Trajectory parkGreen = drive.trajectoryBuilder(moveToSignalCone.end())
                .strafeRight(24)
                .build();

        robot.armPortI.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armPortO.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armStarI.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armStarO.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.armPortI.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.armPortO.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.armStarI.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.armStarO.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        if (isStopRequested()) return;

        PIDArmControl(twoConeArmTarget, 3);
        drive.followTrajectory(moveToSignalCone);

        if(senseColorsStar().equals("blue")){
            drive.followTrajectory(parkBlue);
        } else if(senseColorsStar().equals("green")){
            drive.followTrajectory(parkGreen);
        } else {
            drive.followTrajectory(parkRed);
        }
    }

    public void PIDArmControl(double targetArm, double timeout) {
        armPID.setPID(pArm, iArm, dArm);
        armPID.setSetPoint(targetArm);
        armPID.setTolerance(20);

        double robotArmStar = robot.armStarI.getCurrentPosition();

        resetRuntime();
        while ((!armPID.atSetPoint()) && getRuntime() < timeout) {
            robotArmStar = robot.armStarI.getCurrentPosition();

            double pidArm = armPID.calculate(robotArmStar, armPID.getSetPoint());

            double powerArm = pidArm;

            robot.armPortI.setPower(powerArm);
            robot.armPortO.setPower(powerArm);
            robot.armStarI.setPower(powerArm);
            robot.armStarO.setPower(powerArm);

        }
        robot.armPortI.setPower(0);
        robot.armPortO.setPower(0);
        robot.armStarI.setPower(0);
        robot.armStarO.setPower(0);
    }

    public String senseColorsStar() {
        String colorStar = "blank";

        while (opModeIsActive() && colorStar.equals("blank")) {
            if (robot.colorSensorStar.red() > ((robot.colorSensorStar.blue()) - 5) && robot.colorSensorStar.red() > ((robot.colorSensorStar.green())) - 20) {
                colorStar = "red";
                telemetry.addData("i see red", " ");
                telemetry.update();
                colorStar = "red";
                //sleeveColor.equals(red);

            } else if (robot.colorSensorStar.blue() > (robot.colorSensorStar.red()) && robot.colorSensorStar.blue() > (robot.colorSensorStar.green())) {
                colorStar = "blue";
                telemetry.addData("i see blue", " ");
                telemetry.update();
                colorStar = "blue";

            } else if (robot.colorSensorStar.green() > (robot.colorSensorStar.red()) && robot.colorSensorStar.green() > (robot.colorSensorStar.blue())) {
                colorStar = "green";
                telemetry.addData("i see green", " ");
                telemetry.update();
                colorStar = "green";

            } else {
                telemetry.addData("i see nothing", " ");
                telemetry.update();
                colorStar = "no go";

            }

        }
        return colorStar;
    }
}