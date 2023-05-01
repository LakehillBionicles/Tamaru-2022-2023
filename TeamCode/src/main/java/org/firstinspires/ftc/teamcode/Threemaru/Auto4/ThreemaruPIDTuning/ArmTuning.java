package org.firstinspires.ftc.teamcode.Threemaru.Auto4.ThreemaruPIDTuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Threemaru.Tele4.ThreemaruHardware;

@Config
@TeleOp
public class ArmTuning extends OpMode {
    ThreemaruHardware robot = new ThreemaruHardware();
    private PIDController controller;

    public static double p = 0, i = 0, d = 0, kg = 0;
    public static int reference = 0;
    public static double maxVelocity = 4000;

    @Override
    public void init(){
        robot.init(hardwareMap);
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.armPort.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armStar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.armPort.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.armStar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop(){
        controller.setPID(p, i, d);

        double state = (robot.armPort.getCurrentPosition()+robot.armStar.getCurrentPosition()/2.0)/ ThreemaruHardware.COUNTS_PER_INCH;
        double pid = controller.calculate(state, reference) + kg;
        double velocity = pid * maxVelocity;

        robot.armPort.setVelocity(velocity);
        robot.armStar.setVelocity(velocity);

        telemetry.addData("reference", reference);
        telemetry.addData("state", state);
        telemetry.addData("velocity", velocity);
        telemetry.update();
    }
}