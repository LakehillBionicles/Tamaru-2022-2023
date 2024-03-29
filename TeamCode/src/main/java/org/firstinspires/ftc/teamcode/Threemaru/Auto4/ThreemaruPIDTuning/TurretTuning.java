package org.firstinspires.ftc.teamcode.Threemaru.Auto4.ThreemaruPIDTuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import static org.firstinspires.ftc.teamcode.Threemaru.Subsystems.HandSubsystem.HandPos.*;

import org.firstinspires.ftc.teamcode.Threemaru.ThreemaruHardware;

@Disabled
@Config
@TeleOp
public class TurretTuning extends OpMode {
    ThreemaruHardware robot = new ThreemaruHardware();
    private PIDController controller;

    public static double p = 0.005, i = 0, d = 0.00005;
    public static int reference = 0;
    public static double maxVelocity = 4000;

    @Override
    public void init(){
        robot.init(hardwareMap);
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.motorTurret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motorTurret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.servoHand1.setPosition(CLOSED1.getPosition());
        robot.servoHand2.setPosition(CLOSED2.getPosition());
    }

    @Override
    public void loop(){
        controller.setPID(p, i, d);

        double state = robot.motorTurret.getCurrentPosition();
        double pid = controller.calculate(state, reference);
        double velocity = pid * maxVelocity;

        robot.motorTurret.setVelocity(velocity);

        telemetry.addData("reference", reference);
        telemetry.addData("state", state);
        telemetry.addData("velocity", velocity);
        telemetry.update();
    }
}