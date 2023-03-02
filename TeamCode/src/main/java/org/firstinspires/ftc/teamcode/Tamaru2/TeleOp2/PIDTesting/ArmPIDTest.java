package org.firstinspires.ftc.teamcode.Tamaru2.TeleOp2.PIDTesting;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor; //DcMotorEx?

import org.firstinspires.ftc.teamcode.Tamaru2.BaseClasses.Tamaru2Hardware;

@Config
@TeleOp
public class ArmPIDTest extends OpMode{
    Tamaru2Hardware robot = new Tamaru2Hardware();
    private PIDController armController;

    public static double p = 0, i = 0, d = 0;
    public static double f = 0;

    public static double target = 0;

    public final double COUNTS_PER_ODO_REV = 8192;
    public final double ODO_GEAR_REDUCTION = (1.0); // This is < 1.0 if geared UP
    public final double ODO_WHEEL_DIAMETER_INCHES = 2.0;  // For figuring circumference
    public final double ODO_COUNTS_PER_INCH = ((COUNTS_PER_ODO_REV * ODO_GEAR_REDUCTION) /
            (ODO_WHEEL_DIAMETER_INCHES * 3.1415));


    @Override
    public void init(){
        robot.init(hardwareMap);
        armController = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.fpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.fsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.fpd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.bpd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.fsd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.bsd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.armPort.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armStar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.armPort.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.armStar.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.servoHand.setPosition(robot.handClosed);
    }

    @Override
    public void loop(){
        robot.servoHand.setPosition(robot.handClosed);

        armController.setPID(p, i, d);
        double armPort = (robot.armPort.getCurrentPosition());
        double armStar = (robot.armStar.getCurrentPosition());
        double armPos = (armPort+armStar)/2;
        double pid = armController.calculate(armPos, target);
        double ff = (target)*f;

        double power = (pid + ff);

        robot.armPort.setPower(power);
        robot.armStar.setPower(power);

        telemetry.addData("armPos", armPos);
        telemetry.addData("target", target);
        telemetry.update();
    }
}
