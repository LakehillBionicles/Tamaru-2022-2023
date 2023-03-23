package org.firstinspires.ftc.teamcode.Tamaru3.Auto3;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Tamaru3.Tamaru3Hardware;
import org.firstinspires.ftc.teamcode.CoordinateBased.field.*;


@Config
public class AutoBase extends LinearOpMode {
    public Tamaru3Hardware robot = new Tamaru3Hardware();

    private PIDController armPID;
    public static double pArm = 0.01, iArm = 0.0001, dArm = 0.0002;

    public final int downArmTarget = 0, lowPoleArmTarget = 1100, midPoleArmTarget = 2000, highPoleArmTarget = 2800;
    public final int fiveConeArmTarget = 500, fourConeArmTarget = 350, threeConeArmTarget = 250, twoConeArmTarget = 150;
    private Coordinates targetCoordinates;

    public String sleeveColor = "";

    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
        armPID = new PIDController(pArm, iArm, dArm);
        resetArm();
        resetDrive();
    }

    public String senseColorsFront() {
        String colorFront = "blank";

        while (opModeIsActive() && colorFront.equals("blank")) {
            if (robot.colorSensorFront.red() > ((robot.colorSensorFront.blue()) - 5) && robot.colorSensorFront.red() > ((robot.colorSensorFront.green())) - 20) {
                colorFront = "red";
                telemetry.addData("i see red", " ");
                telemetry.update();
                colorFront = "red";
                //sleeveColor.equals(red);

            } else if (robot.colorSensorFront.blue() > (robot.colorSensorFront.red()) && robot.colorSensorFront.blue() > (robot.colorSensorFront.green())) {
                colorFront = "blue";
                telemetry.addData("i see blue", " ");
                telemetry.update();
                colorFront = "blue";

            } else if (robot.colorSensorFront.green() > (robot.colorSensorFront.red()) && robot.colorSensorFront.green() > (robot.colorSensorFront.blue())) {
                colorFront = "green";
                telemetry.addData("i see green", " ");
                telemetry.update();
                colorFront = "green";

            } else {
                telemetry.addData("i see nothing", " ");
                telemetry.update();
                colorFront = "no go";

            }

        }
        return colorFront;
    }

    /*public String scanCone() {
        if (senseColorsFront().equals("blue")) {
            sleeveColor = "blue";
        } else if (senseColorsFront().equals("green")) {
            sleeveColor = "green";
        } else {
            sleeveColor = "red";
        }
        return sleeveColor;
    }*/

    //TODO: add some sort of timeout in case something goes wrong with the touch sensors
    public void correctAngle(){
        while(!robot.touchSensorPort.isPressed()||!robot.touchSensorStar.isPressed()) {
            while (!robot.touchSensorPort.isPressed() && robot.touchSensorStar.isPressed()) {
                robot.fpd.setPower(.25);
                robot.bpd.setPower(.25);
                robot.fsd.setPower(-.25);
                robot.bsd.setPower(-.25);
            }
            while (!robot.touchSensorStar.isPressed() && robot.touchSensorPort.isPressed()) {
                robot.fpd.setPower(-.25);
                robot.bpd.setPower(-.25);
                robot.fsd.setPower(.25);
                robot.bsd.setPower(.25);
            }
            while (!robot.touchSensorPort.isPressed() && !robot.touchSensorStar.isPressed()) {
                robot.fpd.setPower(.5);
                robot.bpd.setPower(.5);
                robot.fsd.setPower(.5);
                robot.bsd.setPower(.5);
            }
        }
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

    public void extensionToPosition(double extensionPosition) {
        robot.servoExtend.setPosition(extensionPosition);
    }

}