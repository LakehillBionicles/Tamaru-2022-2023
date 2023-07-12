package org.firstinspires.ftc.teamcode.Threemaru2.PIDTuning;

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
import org.firstinspires.ftc.teamcode.Threemaru2.Threemaru2Hardware;

//@Disabled
@Config
@TeleOp
public class TuningAll extends OpMode{
    Threemaru2Hardware robot = new Threemaru2Hardware();
    private PIDController yController;
    private PIDController xController;
    private PIDController thetaController;

    BNO055IMU imu;
    Orientation robotTheta;

    public static double py = 0.0, iy = 0.0, dy = 0.0;
    public static double px = 0.0, ix = 0.0, dx = 0.0;
    public static double pTheta = 0.4, iTheta = 0.0, dTheta = 0.0005;

    public static double targetY = 0;
    public static double targetX = 0;
    public static double targetTheta = 0;

    public static double max = 4000;
    public static double xMultiplier = 1;

    public final double COUNTS_PER_ODO_REV = 8192;
    public final double ODO_GEAR_REDUCTION = (1.0); // This is < 1.0 if geared UP
    public final double ODO_WHEEL_DIAMETER_INCHES = 2.0;  // For figuring circumference
    public final double ODO_COUNTS_PER_INCH = ((COUNTS_PER_ODO_REV * ODO_GEAR_REDUCTION) /
            (ODO_WHEEL_DIAMETER_INCHES * 3.1415));

    public final double odoWheelGap = 2.25;

    @Override
    public void init(){
        robot.init(hardwareMap);
        yController = new PIDController(py, iy, dy);
        xController = new PIDController(px, ix, dx);
        thetaController = new PIDController(pTheta, iTheta, dTheta);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        robot.POW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.SOW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BOWF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BOWB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.POW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.SOW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BOWF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BOWB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop(){
        yController.setPID(py, iy, dy);
        xController.setPID(px, ix, dx);
        thetaController.setPID(pTheta, iTheta, dTheta);

        int POW = robot.POW.getCurrentPosition();
        int SOW = robot.SOW.getCurrentPosition();
        int BOWF = robot.BOWF.getCurrentPosition();
        int BOWB = robot.BOWB.getCurrentPosition();

        double robotY = ((POW+SOW)/2.0)/ODO_COUNTS_PER_INCH;
        robotTheta = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double robotX = -1*((((BOWF+BOWB)/2.0) / ODO_COUNTS_PER_INCH) - (2.5 * (POW-SOW)/ODO_COUNTS_PER_INCH/odoWheelGap));

        double pidY = yController.calculate(robotY, targetY);
        double pidX = xController.calculate(robotX, targetX);
        double pidTheta = thetaController.calculate(robotTheta.firstAngle, targetTheta);

        robot.fpd.setVelocity(pidY + pidX - pidTheta);
        robot.bpd.setVelocity(pidY - pidX - pidTheta);
        robot.fsd.setVelocity(pidY - pidX + pidTheta);
        robot.bsd.setVelocity(pidY + pidX + pidTheta);

        telemetry.addData("robotY", robotY);
        telemetry.addData("robotX", robotX);
        telemetry.addData("robotTheta", robotTheta.firstAngle);
        telemetry.addData("targetY", targetY);
        telemetry.addData("targetX", targetX);
        telemetry.addData("targetTheta", targetTheta);
        telemetry.update();
    }
}
