package org.firstinspires.ftc.teamcode.Tamaru3.TeleOp3.PIDTuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor; //DcMotorEx?

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Tamaru3.Tamaru3Hardware;

@Config
@TeleOp
public class PIDTuningAll extends OpMode{
    Tamaru3Hardware robot = new Tamaru3Hardware();
    private PIDController yController;
    private PIDController xController;
    private PIDController thetaController;
    private PIDController armController;

    BNO055IMU imu;
    Orientation robotTheta;

    /* ODO Gains
    public static double py = 0.05, iy = 0.0001, dy = 0.015;
    public static double px = 0.1, ix = 0.001, dx = 0.025;
    public static double pTheta = 0.9, iTheta = 0.0, dTheta = 0.05;*/

    public static double py = 0.075, iy = 0.0005, dy = 0.01;
    public static double px = 0.01, ix = 0.001, dx = 0.01;
    public static double pTheta = 0.01, iTheta = 0.0, dTheta = 0.001;
    public static double pArm = 0.075, iArm = 0.0, dArm = 0.0005;

    public static double targetY = 0;
    public static double targetX = 0;
    public static double targetTheta = 0;
    public static double targetArm = 0;

    public static double handPos = 0;
    public static double extensionPos = 0;
    public static double turretPos = .6;

    //public static double max = 2500;
    public static double max = 4000;
    public static double xMultiplier = 1;

    public final double COUNTS_PER_ODO_REV = 8192;
    public final double ODO_GEAR_REDUCTION = (1.0); // This is < 1.0 if geared UP
    public final double ODO_WHEEL_DIAMETER_INCHES = 2.0;  // For figuring circumference
    public final double ODO_COUNTS_PER_INCH = ((COUNTS_PER_ODO_REV * ODO_GEAR_REDUCTION) /
            (ODO_WHEEL_DIAMETER_INCHES * 3.1415));

    public final double WHEEL_COUNTS_PER_INCH = 22.48958;

    public final double odoWheelGap = 12.5;


    @Override
    public void init(){
        robot.init(hardwareMap);
        yController = new PIDController(py, iy, dy);
        xController = new PIDController(px, ix, dx);
        thetaController = new PIDController(pTheta, iTheta, dTheta);
        armController = new PIDController(pArm, iArm, dArm);
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

        robot.armPortI.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armPortO.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armStarI.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armStarO.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.armPortI.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.armPortO.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.armStarI.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.armStarO.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop(){
        yController.setPID(py, iy, dy);
        xController.setPID(px, ix, dx);
        thetaController.setPID(pTheta, iTheta, dTheta);
        //armController.setPID(pArm, iArm, dArm);

        //double robotY = ((robot.armPort_POW.getCurrentPosition()+robot.SOW.getCurrentPosition())/2)/ODO_COUNTS_PER_INCH;
        //double robotTheta = (robot.armPort_POW.getCurrentPosition()-robot.SOW.getCurrentPosition())/ODO_COUNTS_PER_INCH / odoWheelGap;
        //double robotX = (robot.BOW.getCurrentPosition() / ODO_COUNTS_PER_INCH) - (2.5 * robotTheta);

        /*int portAvg = (robot.fpd.getCurrentPosition() + robot.bpd.getCurrentPosition()) / 2;
        int starAvg = (robot.fsd.getCurrentPosition()+robot.bsd.getCurrentPosition())/2;
        double robotY = ((portAvg+starAvg)/2)/WHEEL_COUNTS_PER_INCH;
        robotTheta = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double robotX = -1*((robot.armPortI.getCurrentPosition() / ODO_COUNTS_PER_INCH) - (2.5 * (portAvg-starAvg)/WHEEL_COUNTS_PER_INCH/odoWheelGap));*/
        int POW = (robot.bpd.getCurrentPosition()); //TODO: check that these are the right motors for the odowheels
        int BOW = robot.fpd.getCurrentPosition();
        int SOW = robot.bsd.getCurrentPosition();
        double robotY = ((POW+SOW)/2)/WHEEL_COUNTS_PER_INCH;
        robotTheta = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double robotX = -1*((BOW / ODO_COUNTS_PER_INCH) - (2.5 * (POW-SOW)/WHEEL_COUNTS_PER_INCH/odoWheelGap));

        double robotArm = robot.armStarI.getCurrentPosition();

        double pidY = yController.calculate(robotY, targetY);
        double pidX = xController.calculate(robotX, targetX);
        double pidTheta = thetaController.calculate(robotTheta.firstAngle, targetTheta);
        //double pidArm = armController.calculate(robotArm, targetArm);

        double yVelocity = pidY * max;
        double xVelocity = -pidX * max;
        double thetaVelocity = -pidTheta * max;
        //double armPower = pidArm;

        robot.fpd.setVelocity(yVelocity + xMultiplier*xVelocity + thetaVelocity);
        robot.bpd.setVelocity(yVelocity - xMultiplier*xVelocity + thetaVelocity);
        robot.fsd.setVelocity(yVelocity - xMultiplier*xVelocity - thetaVelocity);
        robot.bsd.setVelocity(yVelocity + xMultiplier*xVelocity - thetaVelocity);

        telemetry.addData("robotY", robotY);
        telemetry.addData("robotX", robotX);
        telemetry.addData("robotTheta", robotTheta.firstAngle);
        telemetry.addData("robotArm", robotArm);
        telemetry.update();
    }
}
