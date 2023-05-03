package org.firstinspires.ftc.teamcode.Threemaru.Auto4.ThreemaruPIDTuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Tamaru3.Tamaru3Hardware;
import org.firstinspires.ftc.teamcode.Threemaru.Tele4.ThreemaruHardware;

@Disabled
@Config
@TeleOp
public class YTuning extends OpMode{
    ThreemaruHardware robot = new ThreemaruHardware();
    private PIDController controller;

    public static double p = 0.05, i = 0, d = 0.001;
    public static int target = 0;
    public static double maxVelocity = 4000;

    @Override
    public void init(){
        robot.init(hardwareMap);
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

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
        controller.setPID(p, i, d);
        double robotY = ((robot.fpd.getCurrentPosition()+robot.bpd.getCurrentPosition() +robot.fsd.getCurrentPosition()+robot.bsd.getCurrentPosition())/4.0) / ThreemaruHardware.COUNTS_PER_INCH;

        double pidY = controller.calculate(robotY, target);

        double velocityY = pidY * maxVelocity;

        robot.fpd.setVelocity(velocityY);
        robot.bpd.setVelocity(velocityY);
        robot.fsd.setVelocity(velocityY);
        robot.bsd.setVelocity(velocityY);

        telemetry.addData("robotY", robotY);
        telemetry.addData("target", target);
        telemetry.addData("velocityY", velocityY);
        telemetry.update();
    }
}