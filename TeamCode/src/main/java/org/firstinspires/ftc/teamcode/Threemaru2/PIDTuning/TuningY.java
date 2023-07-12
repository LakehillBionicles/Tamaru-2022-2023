package org.firstinspires.ftc.teamcode.Threemaru2.PIDTuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Threemaru2.Threemaru2Hardware;

//@Disabled
@Config
@TeleOp
public class TuningY extends OpMode{
    Threemaru2Hardware robot = new Threemaru2Hardware();
    private PIDController controller;

    public static double p = 0, i = 0, d = 0;

    public static int target = 0;

    public static double maxVelocity = 4000;


    public final double COUNTS_PER_ODO_REV = 8192;
    public final double ODO_GEAR_REDUCTION = (1.0); // This is < 1.0 if geared UP
    public final double ODO_WHEEL_DIAMETER_INCHES = 2.0;  // For figuring circumference
    public final double ODO_COUNTS_PER_INCH = ((COUNTS_PER_ODO_REV * ODO_GEAR_REDUCTION) /
            (ODO_WHEEL_DIAMETER_INCHES * 3.1415));

    public final double odoWheelGap = 2.25;

    public final double wheelGap = 12.5;


    @Override
    public void init(){
        robot.init(hardwareMap);
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

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
        controller.setPID(p, i, d);
        int POW = robot.POW.getCurrentPosition();
        int SOW = robot.SOW.getCurrentPosition();
        int BOWF = robot.BOWF.getCurrentPosition();
        int BOWB = robot.BOWB.getCurrentPosition();

        double robotY = ((POW+SOW)/2.0)/ODO_COUNTS_PER_INCH;

        double pidY = controller.calculate(robotY, target);

        robot.fpd.setVelocity(pidY);
        robot.bpd.setVelocity(pidY);
        robot.fsd.setVelocity(pidY);
        robot.bsd.setVelocity(pidY);

        telemetry.addData("robotY", robotY);
        telemetry.addData("target", target);
        telemetry.addData("pidY", pidY);
        telemetry.update();
    }
}
