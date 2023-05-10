package org.firstinspires.ftc.teamcode.Threemaru.Auto4.ThreemaruPIDTuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor; //DcMotorEx?
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Threemaru.ThreemaruHardware;

import java.net.PortUnreachableException;

@Disabled
@Config
@TeleOp
public class DriveTuning extends OpMode{
    ThreemaruHardware robot = new ThreemaruHardware();
    private PIDController thetaController;
    private PIDController yController;

    //IMU imu;
    BNO055IMU imu;
    Orientation robotTheta;

    public static double pTheta = 0.0075, iTheta = 0, dTheta = 0.0012;
    //public static double pY = 0.05, iY = 0, dY = 0.001;
    public static double pY = 0.0275, iY = 0.00055, dY = 0;
    public static int targetTheta = 0;
    public static int targetY = 0;
    public static double maxVelocity = 4000;

    @Override
    public void init(){
        robot.init(hardwareMap);
        thetaController = new PIDController(pTheta, iTheta, dTheta);
        yController = new PIDController(pY, iY, dY);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        robot.fpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.fsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.fpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.fsd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bsd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    @Override
    public void loop(){
        thetaController.setPID(pTheta, iTheta, dTheta);
        yController.setPID(pY, iY, dY);

        robotTheta = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double robotY = ((robot.fpd.getCurrentPosition()+robot.bpd.getCurrentPosition() +robot.fsd.getCurrentPosition()+robot.bsd.getCurrentPosition())/4.0) / ThreemaruHardware.COUNTS_PER_INCH;

        double pidTheta = thetaController.calculate(robotTheta.firstAngle, targetTheta);
        double pidY = yController.calculate(robotY, targetY);
        double velocityTheta = pidTheta * maxVelocity;
        double velocityY = pidY * maxVelocity;

        robot.fpd.setVelocity(velocityY - velocityTheta);
        robot.bpd.setVelocity(velocityY - velocityTheta);
        robot.fsd.setVelocity(velocityY + velocityTheta);
        robot.bsd.setVelocity(velocityY + velocityTheta);

        telemetry.addData("robotTheta", robotTheta.firstAngle);
        telemetry.addData("targetTheta", targetTheta);
        telemetry.addData("robotY", robotY);
        telemetry.addData("targetY", targetY);
        telemetry.update();
    }
}