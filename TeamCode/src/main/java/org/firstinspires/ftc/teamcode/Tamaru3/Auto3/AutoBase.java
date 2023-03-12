package org.firstinspires.ftc.teamcode.Tamaru3.Auto3;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Tamaru3.Tamaru3Hardware;

import org.firstinspires.ftc.teamcode.CoordinateBased.field.*;
import org.firstinspires.ftc.teamcode.CoordinateBased.field;

import java.util.HashMap;

@Config
public class AutoBase extends LinearOpMode {
    public Tamaru3Hardware robot = new Tamaru3Hardware();
    public field field;
    public field.Coordinates currentCoordinates;
    public SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    private PIDController armPID;
    public static double pArm = 0.01, iArm = 0.0001, dArm = 0.0002;

    public final int downArmTarget = 0, lowPoleArmTarget = 1400, midPoleArmTarget = 2000, highPoleArmTarget = 2600;
    public final int fiveConeArmTarget = 600, fourConeArmTarget = 500, threeConeArmTarget = 400, twoConeArmTarget = 300;
    private Coordinates targetCoordinates;

    /*public AutoBase(Location location){
        field = new field(location);
        currentCoordinates = new field.Coordinates(0, 0, Height.GROUND);
    }*/

    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        armPID = new PIDController(pArm, iArm, dArm);
        resetArm();
        resetDrive();
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

    public void resetArm(){
        robot.armPortI.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armPortO.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armStarI.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armStarO.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.armPortI.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.armPortO.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.armStarI.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.armStarO.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetDrive(){
        robot.fpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.fsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.fpd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.bpd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.fsd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.bsd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void armToPosition(int position){
        robot.armPortI.setTargetPosition(position);
        robot.armPortO.setTargetPosition(position);
        robot.armStarI.setTargetPosition(position);
        robot.armStarO.setTargetPosition(position);

        robot.armPortI.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armPortO.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armStarI.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armStarO.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.armPortI.setPower(1);
        robot.armPortO.setPower(1);
        robot.armStarI.setPower(1);
        robot.armStarO.setPower(1);
    }

    public void PIDArmControl(double targetArm, double timeout) {
        armPID.setPID(pArm, iArm, dArm);
        armPID.setSetPoint(targetArm);
        armPID.setTolerance(20);

        resetRuntime();
        while ((!armPID.atSetPoint()) && getRuntime() < timeout) {
            double robotArmStar = robot.armStarI.getCurrentPosition();

            double powerArm = armPID.calculate(robotArmStar, armPID.getSetPoint());

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

    public void turretToPosition(double turretPosition) {
        robot.servoTurret.setPosition(turretPosition);
    }

    /*public void Score(String id, field.Coordinates currentCoordinates){
        this.currentCoordinates = currentCoordinates;
        targetCoordinates = field.scoreMap.get(id);
        double targetX = targetCoordinates.x;
        double targetY = targetCoordinates.y;
        field.armScore(id, currentCoordinates);

        TrajectorySequence scoreTraj = drive.trajectoryBuilder(new Pose2d(currentCoordinates.x, currentCoordinates.y, 0))
                .addDisplacementMarker(() -> { field.armScore(id, currentCoordinates); })
                .turn(Math.toRadians(90))
                .lineTo()
                .strafeTo()
                .build();
    }*/

}
