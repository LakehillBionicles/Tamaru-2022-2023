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
public class PIDTestThetaVelocity extends OpMode{
    Tamaru2Hardware robot = new Tamaru2Hardware();
    private PIDController controller;

    public static double p = 0, i = 0, d = 0;

    public static int target = 0;

    public final double COUNTS_PER_ODO_REV = 8192;
    public final double ODO_GEAR_REDUCTION = (1.0); // This is < 1.0 if geared UP
    public final double ODO_WHEEL_DIAMETER_INCHES = 2.0;  // For figuring circumference
    public final double ODO_COUNTS_PER_INCH = ((COUNTS_PER_ODO_REV * ODO_GEAR_REDUCTION) /
            (ODO_WHEEL_DIAMETER_INCHES * 3.1415));

    public final double odoWheelGap = 12.5;//used to be 11.5

    @Override
    public void init(){
        robot.init(hardwareMap);
        controller = new PIDController(p, i, d);
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
        controller.setPID(p, i, d);

        double robotTheta = ((robot.fpd.getCurrentPosition()-robot.fsd.getCurrentPosition())/ODO_COUNTS_PER_INCH / odoWheelGap);

        double pidTheta = controller.calculate(robotTheta, Math.toRadians(target));

        double velocityTheta = -pidTheta * robot.maxVelocity;

        robot.fpd.setVelocity(velocityTheta);
        robot.bpd.setVelocity(velocityTheta);
        robot.fsd.setVelocity(-velocityTheta);
        robot.bsd.setVelocity(-velocityTheta);

        telemetry.addData("robotTheta", Math.toDegrees(robotTheta));
        telemetry.addData("target", target);
        telemetry.update();
    }
}
