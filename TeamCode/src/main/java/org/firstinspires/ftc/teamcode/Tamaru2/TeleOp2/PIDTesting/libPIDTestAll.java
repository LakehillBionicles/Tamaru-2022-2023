package org.firstinspires.ftc.teamcode.Tamaru2.TeleOp2.PIDTesting;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor; //DcMotorEx?

import org.firstinspires.ftc.teamcode.Tamaru2.BaseClasses.Tamaru2Hardware;

@Disabled
@Config
@TeleOp
public class libPIDTestAll extends OpMode{
    Tamaru2Hardware robot = new Tamaru2Hardware();
    private PIDController yController;
    private PIDController xController;
    private PIDController thetaController;

    public static double py = 0.0275, iy = 0.00055, dy = 0;
    public static double fy = 0;
    public static double px = 0.075, ix = 0.0001, dx = 0;
    public static double fx = 0;
    public static double pTheta = 0.855, iTheta = 0.01, dTheta = 0;
    public static double fTheta = 0.05;

    public static double targetY = 0;
    public static double targetX = 0;
    public static double targetTheta = 0;


    public final double COUNTS_PER_ODO_REV = 8192;
    public final double ODO_GEAR_REDUCTION = (1.0); // This is < 1.0 if geared UP
    public final double ODO_WHEEL_DIAMETER_INCHES = 2.0;  // For figuring circumference
    public final double ODO_COUNTS_PER_INCH = ((COUNTS_PER_ODO_REV * ODO_GEAR_REDUCTION) /
            (ODO_WHEEL_DIAMETER_INCHES * 3.1415));

    public final double odoWheelGap = 12.5;//used to be 11.5


    @Override
    public void init(){
        robot.init(hardwareMap);
        yController = new PIDController(py, iy, dy);
        xController = new PIDController(px, ix, dx);
        thetaController = new PIDController(pTheta, iTheta, dTheta);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.fpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.fsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.fpd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.bpd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.fsd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.bsd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop(){
        yController.setPID(py, iy, dy);
        xController.setPID(px, ix, dx);
        thetaController.setPID(pTheta, iTheta, dTheta);

        double robotY = ((robot.fpd.getCurrentPosition()+robot.fsd.getCurrentPosition())/2)/ODO_COUNTS_PER_INCH;
        double robotTheta = (robot.fpd.getCurrentPosition()-robot.fsd.getCurrentPosition())/ODO_COUNTS_PER_INCH / odoWheelGap;
        double robotX = (robot.bpd.getCurrentPosition() / ODO_COUNTS_PER_INCH) - (2.5 * robotTheta);

        double pidY = yController.calculate(robotY, targetY);
        double pidX = xController.calculate(robotX, targetX);
        double pidTheta = thetaController.calculate(robotTheta, Math.toRadians(targetTheta));

        double ffY = (targetY/ODO_COUNTS_PER_INCH)*fy;
        double ffX = (targetX/ODO_COUNTS_PER_INCH)*fx;
        double ffTheta = (targetTheta/ODO_COUNTS_PER_INCH)*fTheta;

        double yPower = -(pidY + ffY);
        double xPower = -(pidX + ffX);
        double thetaPower = -(pidTheta + ffTheta);

        robot.fpd.setPower(yPower - xPower + thetaPower);
        robot.bpd.setPower(yPower + xPower + thetaPower);
        robot.fsd.setPower(yPower + xPower - thetaPower);
        robot.bsd.setPower(yPower - xPower - thetaPower);

        telemetry.addData("robotY", robotY);
        telemetry.addData("robotX", robotX);
        telemetry.addData("robotTheta", Math.toDegrees(robotTheta));
        telemetry.addData("targetY", targetY);
        telemetry.addData("targetX", targetX);
        telemetry.addData("targetTheta", targetTheta);
        telemetry.update();
    }
}
