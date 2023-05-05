package org.firstinspires.ftc.teamcode.Tamaru3.Auto3;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Tamaru3.Tamaru3Hardware;
import org.firstinspires.ftc.teamcode.CoordinateBased.field.*;


@Config
public class AutoBase extends LinearOpMode {
    public Tamaru3Hardware robot = new Tamaru3Hardware();

    private PIDController armPID;
    public static double pArm = 0.01, iArm = 0.0001, dArm = 0.0002;

    private PIDController yController;
    private PIDController xController;
    private PIDController thetaController;
    private PIDController armController;

    public static double py = 0.0, iy = 0.0, dy = 0.0;
    public static double px = 0.0, ix = 0.0, dx = 0.0;
    public static double pTheta = 0.0, iTheta = 0.0, dTheta = 0.0;

    public static double max = 2500;
    public static double xMultiplier = 1;

    public final double COUNTS_PER_ODO_REV = 8192;
    public final double ODO_GEAR_REDUCTION = (1.0); // This is < 1.0 if geared UP
    public final double ODO_WHEEL_DIAMETER_INCHES = 2.0;  // For figuring circumference
    public final double ODO_COUNTS_PER_INCH = ((COUNTS_PER_ODO_REV * ODO_GEAR_REDUCTION) /
            (ODO_WHEEL_DIAMETER_INCHES * 3.1415));

    public final double WHEEL_COUNTS_PER_INCH = 22.48958;

    public final double odoWheelGap = 12.5;

    BNO055IMU imu;
    Orientation robotTheta;

    public final int downArmTarget = 0, lowPoleArmTarget = 1200, midPoleArmTarget = 2000, highPoleArmTarget = 2800;
    public final int fiveConeArmTarget = 450, fourConeArmTarget = 350, threeConeArmTarget = 250, twoConeArmTarget = 150;
    private Coordinates targetCoordinates;

    public String sleeveColor = "";

    @Override
    public void runOpMode(){
        robot.init(hardwareMap);

        armPID = new PIDController(pArm, iArm, dArm);
        yController = new PIDController(py, iy, dy);
        xController = new PIDController(px, ix, dx);
        thetaController = new PIDController(pTheta, iTheta, dTheta);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        resetArm();
        resetDrive();
    }

    public String senseColorsFront() {
        String colorFront = "blank";
        double redMax = 1.5*Math.max(Math.max(robot.colorSensorFront.red(), robot.colorSensorPortBottom.red()), robot.colorSensorStarBottom.red());
        int blueMax = Math.max(Math.max(robot.colorSensorFront.blue(), robot.colorSensorPortBottom.blue()), robot.colorSensorStarBottom.blue());
        int greenMax = Math.max(Math.max(robot.colorSensorFront.green(), robot.colorSensorPortBottom.green()), robot.colorSensorStarBottom.green());

        while (opModeIsActive() && colorFront.equals("blank")) {
            if ((redMax > blueMax) && (redMax > greenMax)) {
                colorFront = "red";
            } else if ((blueMax > redMax) && (blueMax > greenMax)) {
                colorFront = "blue";
            } else if ((greenMax > redMax) && (greenMax > blueMax)) {
                colorFront = "green";
            } else {
                colorFront = "no go";
            }

        }
        return colorFront;
    }

    public void lineUpWithConeStackPickUpRight(String color){
        resetRuntime();
        while(!senseColors(robot.colorSensorBottom).equals(color) && getRuntime()<1){
            robot.fpd.setPower(.5);
            robot.bpd.setPower(-.5);
            robot.fsd.setPower(-.5);
            robot.bsd.setPower(.5);
        }
        robot.fpd.setPower(0);
        robot.bpd.setPower(0);
        robot.fsd.setPower(0);
        robot.bsd.setPower(0);
    }

    public void lineUpWithConeStackPickUpLeft(String color){
        resetRuntime();
        while(senseColors(robot.colorSensorBottom).equals(color) && getRuntime()<1){
            robot.fpd.setPower(-.5);
            robot.bpd.setPower(.5);
            robot.fsd.setPower(.5);
            robot.bsd.setPower(-.5);
        }
        robot.fpd.setPower(0);
        robot.bpd.setPower(0);
        robot.fsd.setPower(0);
        robot.bsd.setPower(0);
    }

    public void lineUpWithConeStackScore(String color){
        resetRuntime();
        while(!senseColors(robot.colorSensorFront).equals(color) && getRuntime()<.5){
            robot.fpd.setPower(-.5);
            robot.bpd.setPower(.5);
            robot.fsd.setPower(.5);
            robot.bsd.setPower(-.5);
        }
        robot.fpd.setPower(0);
        robot.bpd.setPower(0);
        robot.fsd.setPower(0);
        robot.bsd.setPower(0);
    }

    public String senseColors(ColorSensor colorSensor) {
        String color = "blank";
        double redMax = colorSensor.red();
        int blueMax = colorSensor.blue();
        int greenMax = colorSensor.green();

        while (opModeIsActive() && color.equals("blank")) {
            if ((redMax > blueMax) && (redMax > greenMax)) {
                telemetry.addData("i see red", " ");
                telemetry.update();
                color = "red";
            } else if ((blueMax > redMax) && (blueMax > greenMax)) {
                telemetry.addData("i see blue", " ");
                telemetry.update();
                color = "blue";
            } else if ((greenMax > redMax) && (greenMax > blueMax)) {
                telemetry.addData("i see green", " ");
                telemetry.update();
                color = "green";
            } else {
                telemetry.addData("i see nothing", " ");
                telemetry.update();
                color = "no go";
            }

        }
        return color;
    }

    public void correctAngle(){
        resetRuntime();
        while((!robot.touchSensorPort.isPressed()||!robot.touchSensorStar.isPressed())&&getRuntime()<.75) {
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

    public void PIDDrive(double targetX, double targetY, double targetTheta){
        yController.setPID(py, iy, dy);
        xController.setPID(px, ix, dx);
        thetaController.setPID(pTheta, iTheta, dTheta);

        int POW = (robot.fpd.getCurrentPosition()); //TODO: check that these are the right motors for the odowheels
        int BOW = robot.bpd.getCurrentPosition();
        int SOW = robot.fsd.getCurrentPosition();
        double robotY = ((POW+SOW)/2)/WHEEL_COUNTS_PER_INCH;
        robotTheta = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); //TODO: add the imu back into the config
        double robotX = -1*((BOW / ODO_COUNTS_PER_INCH) - (2.5 * (POW-SOW)/WHEEL_COUNTS_PER_INCH/odoWheelGap));

        double pidY = yController.calculate(robotY, targetY);
        double pidX = xController.calculate(robotX, targetX);
        double pidTheta = thetaController.calculate(robotTheta.firstAngle, targetTheta);

        double yVelocity = pidY * max;
        double xVelocity = -pidX * max;
        double thetaVelocity = -pidTheta * max;

        robot.fpd.setVelocity(yVelocity + xMultiplier*xVelocity + thetaVelocity);
        robot.bpd.setVelocity(yVelocity - xMultiplier*xVelocity + thetaVelocity);
        robot.fsd.setVelocity(yVelocity - xMultiplier*xVelocity - thetaVelocity);
        robot.bsd.setVelocity(yVelocity + xMultiplier*xVelocity - thetaVelocity);
    }

    public void distDrivePort(int direction, double timeout){
        resetRuntime();
        while((robot.distSensorPort.getDistance(DistanceUnit.CM)>10||robot.distSensorPort2.getDistance(DistanceUnit.CM)>10)&&getRuntime()<timeout){
            robot.fpd.setPower(direction*.2);
            robot.bpd.setPower(direction*.2);
            robot.fsd.setPower(direction*.2);
            robot.bsd.setPower(direction*.2);

            if(robot.distSensorPort.getDistance(DistanceUnit.CM)<10||robot.distSensorPort2.getDistance(DistanceUnit.CM)<10){
                break;
            }

            telemetry.addData("distPort1", robot.distSensorPort.getDistance(DistanceUnit.CM));
            telemetry.addData("distPort2", robot.distSensorPort2.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
        robot.fpd.setPower(0);
        robot.bpd.setPower(0);
        robot.fsd.setPower(0);
        robot.bsd.setPower(0);
    }

    public void strafeDist(DistanceSensor distSensor, double targetDist, int direction){
        resetRuntime();
        while((distSensor.getDistance(DistanceUnit.CM)>targetDist||distSensor.getDistance(DistanceUnit.CM)>targetDist)&&getRuntime()<2){
            robot.fpd.setPower(direction*.4);
            robot.bpd.setPower(-direction*.4);
            robot.fsd.setPower(-direction*.4);
            robot.bsd.setPower(direction*.4);

            if(distSensor.getDistance(DistanceUnit.CM)<targetDist||distSensor.getDistance(DistanceUnit.CM)<targetDist){
                break;
            }
        }
        robot.fpd.setPower(0);
        robot.bpd.setPower(0);
        robot.fsd.setPower(0);
        robot.bsd.setPower(0);
    }

    public void distDriveStar(int direction, double timeout){
        resetRuntime();
        while(robot.distSensorStar.getDistance(DistanceUnit.CM)>10 && getRuntime()<timeout){
            robot.fpd.setPower(direction*.2);
            robot.bpd.setPower(direction*.2);
            robot.fsd.setPower(direction*.2);
            robot.bsd.setPower(direction*.2);

            telemetry.addData("distStar1", robot.distSensorStar.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
        robot.fpd.setPower(0);
        robot.bpd.setPower(0);
        robot.fsd.setPower(0);
        robot.bsd.setPower(0);
    }
    public void turretToPosition(double turretPosition) {
        robot.servoTurret.setPosition(turretPosition);
    }
    public void extensionToPosition(double extensionPosition) {
        robot.servoExtend.setPosition(extensionPosition);
    }

    public void lights(String color){
        switch (color) {
            case "blue":
                robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                break;
            case "green":
                robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                break;
            case "red":
                robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                break;
            default:
                robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
                break;
        }
    }
}