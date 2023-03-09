package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp
public class VoltageSensorTesting extends LinearOpMode {

    private VoltageSensor batteryVoltageSensor;
    private DcMotor fpd, bpd, fsd, bsd;
    public static double fpdMultiplier=1, bpdMultiplier=1, fsdMultiplier=1, bsdMultiplier=1;

    @Override
    public void runOpMode() {

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        fpd = hardwareMap.get(DcMotor.class, "fpd");
        bpd = hardwareMap.get(DcMotor.class, "bpd");
        fsd = hardwareMap.get(DcMotor.class, "fsd");
        bsd = hardwareMap.get(DcMotor.class, "bsd");

        fpd.setDirection(DcMotorSimple.Direction.REVERSE);
        bpd.setDirection(DcMotorSimple.Direction.FORWARD);
        fsd.setDirection(DcMotorSimple.Direction.REVERSE);
        bsd.setDirection(DcMotorSimple.Direction.FORWARD);

        fpd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bpd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fsd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bsd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fpd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bpd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fsd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bsd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double motorPower = 1;

        fpd.setPower(motorPower*fpdMultiplier);
        bpd.setPower(motorPower*bpdMultiplier);
        fsd.setPower(motorPower*fsdMultiplier);
        bsd.setPower(motorPower*bsdMultiplier);

        telemetry.addData("votage", batteryVoltageSensor.getVoltage());
        telemetry.update();
    }
}
