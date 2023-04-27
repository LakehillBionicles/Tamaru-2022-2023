package org.firstinspires.ftc.teamcode.Threemaru.Auto4;

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
import org.firstinspires.ftc.teamcode.Threemaru.Tele4.ThreemaruHardware;


public class ThreemaruAutoBase extends LinearOpMode {
    public ThreemaruHardware robot = new ThreemaruHardware();

    public final int downArmTarget = 0, lowPoleArmTarget = 1200, midPoleArmTarget = 2000, highPoleArmTarget = 2800;
    public final int fiveConeArmTarget = 450, fourConeArmTarget = 350, threeConeArmTarget = 250, twoConeArmTarget = 150;

    @Override
    public void runOpMode(){
        robot.init(hardwareMap);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        resetArm();
        resetDrive();
    }

    public void encoderDrive(double speed, double leftInches, double rightInches) {
        int leftTarget = (robot.fpd.getCurrentPosition() + robot.bpd.getCurrentPosition())/2 + (int) (leftInches * robot.COUNTS_PER_INCH);
        int rightTarget = (robot.fsd.getCurrentPosition() + robot.bsd.getCurrentPosition())/2 + (int) (rightInches * robot.COUNTS_PER_INCH);

        robot.fpd.setTargetPosition(leftTarget);
        robot.bpd.setTargetPosition(leftTarget);
        robot.fsd.setTargetPosition(rightTarget);
        robot.bsd.setTargetPosition(rightTarget);

        robot.fpd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.bpd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.fsd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.bsd.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.fpd.setPower(1);
        robot.bpd.setPower(1);
        robot.fsd.setPower(1);
        robot.bsd.setPower(1);
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

    public void resetArm(){
        robot.armPort.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armStar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.armPort.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.armStar.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        robot.armPort.setTargetPosition(position);
        robot.armStar.setTargetPosition(position);

        robot.armPort.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armStar.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.armPort.setPower(1);
        robot.armStar.setPower(1);
    }

    public void distDrivePort(int direction, double timeout){
        resetRuntime();
        while((robot.distSensorPort.getDistance(DistanceUnit.CM)>10)&&getRuntime()<timeout){
            robot.fpd.setPower(direction*.2);
            robot.bpd.setPower(direction*.2);
            robot.fsd.setPower(direction*.2);
            robot.bsd.setPower(direction*.2);

            if(robot.distSensorPort.getDistance(DistanceUnit.CM)<10){
                break;
            }

            telemetry.addData("distPort", robot.distSensorPort.getDistance(DistanceUnit.CM));
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
}