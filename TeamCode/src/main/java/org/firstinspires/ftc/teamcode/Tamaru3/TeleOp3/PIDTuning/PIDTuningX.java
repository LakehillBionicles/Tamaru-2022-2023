package org.firstinspires.ftc.teamcode.Tamaru3.TeleOp3.PIDTuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor; //DcMotorEx?

import org.firstinspires.ftc.teamcode.Tamaru3.Tamaru3Hardware;

@Config
@TeleOp
public class PIDTuningX extends OpMode{
    Tamaru3Hardware robot = new Tamaru3Hardware();
    private PIDController controller;

    public static double p = 0, i = 0, d = 0;

    public static int target = 0;

    public final double COUNTS_PER_ODO_REV = 8192;
    public final double ODO_GEAR_REDUCTION = (1.0); // This is < 1.0 if geared UP
    public final double ODO_WHEEL_DIAMETER_INCHES = 2.0;  // For figuring circumference
    public final double ODO_COUNTS_PER_INCH = ((COUNTS_PER_ODO_REV * ODO_GEAR_REDUCTION) /
            (ODO_WHEEL_DIAMETER_INCHES * 3.1415));

    public final double odoWheelGap = 12.5;

    /*public final double COUNTS_PER_WHEEL_REV = 28;//counts @ motor
    public final double WHEEL_GEAR_REDUCTION = (10.4329);
    public final double WHEEL_DIAMETER_INCHES = 3.779;  // For figuring circumference
    public final double WHEEL_COUNTS_PER_INCH = ((COUNTS_PER_WHEEL_REV * WHEEL_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415));*/
    public final double WHEEL_COUNTS_PER_INCH = 22.48958;

    public final double wheelGap = 12.5;

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

        robot.armPort_POW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.SOW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BOW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.armPort_POW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.SOW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.BOW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop(){
        controller.setPID(p, i, d);

        //double robotTheta = (robot.armPort_POW.getCurrentPosition()-robot.SOW.getCurrentPosition())/ODO_COUNTS_PER_INCH / odoWheelGap;
        //double robotX = (robot.BOW.getCurrentPosition() / ODO_COUNTS_PER_INCH) - (2.5 * robotTheta);
        int portAvg = (robot.fpd.getCurrentPosition() + robot.bpd.getCurrentPosition()) / 2;
        int starAvg = (robot.fsd.getCurrentPosition()+robot.bsd.getCurrentPosition())/2;
        double robotY = ((portAvg+starAvg)/2)/WHEEL_COUNTS_PER_INCH;
        double robotTheta = ((portAvg-starAvg)/WHEEL_COUNTS_PER_INCH/wheelGap);
        double robotX = (robot.BOW.getCurrentPosition() / ODO_COUNTS_PER_INCH) - (2.5 * (portAvg-starAvg)/WHEEL_COUNTS_PER_INCH/wheelGap);

        double pidX = controller.calculate(robotX, target);

        double velocityX = -pidX * robot.maxVelocity;

        robot.fpd.setVelocity(velocityX);
        robot.bpd.setVelocity(-velocityX);
        robot.fsd.setVelocity(-velocityX);
        robot.bsd.setVelocity(velocityX);

        telemetry.addData("robotX", robotX);
        telemetry.addData("target", target);
        telemetry.addData("velocityX", velocityX);
        telemetry.addData("fpd", robot.fpd.getCurrentPosition());
        telemetry.addData("bpd", robot.bpd.getCurrentPosition());
        telemetry.addData("fsd", robot.fsd.getCurrentPosition());
        telemetry.addData("bsd", robot.bsd.getCurrentPosition());
        telemetry.update();
    }
}
