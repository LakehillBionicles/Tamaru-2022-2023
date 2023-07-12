package org.firstinspires.ftc.teamcode.Threemaru.Tamaru3.TeleOp3.PIDTuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Threemaru.Tamaru3.Tamaru3Hardware;

@Disabled
@Config
@TeleOp
public class PIDTuningY extends OpMode{
    Tamaru3Hardware robot = new Tamaru3Hardware();
    private PIDController controller;

    public static double p = 0, i = 0, d = 0;

    public static int target = 0;

    public static double maxVelocity = 4000;


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

        robot.armPortI.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armPortO.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armStarI.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armStarO.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.armPortI.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.armPortO.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.armStarI.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.armStarO.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop(){
        controller.setPID(p, i, d);
        //double robotY = ((robot.armPort_POW.getCurrentPosition()+robot.SOW.getCurrentPosition())/2)/ODO_COUNTS_PER_INCH;
        /*int portAvg = (robot.fpd.getCurrentPosition() + robot.bpd.getCurrentPosition()) / 2;
        int starAvg = (robot.fsd.getCurrentPosition()+robot.bsd.getCurrentPosition())/2;
        double robotY = ((portAvg+starAvg)/2)/WHEEL_COUNTS_PER_INCH;
        double robotTheta = ((portAvg-starAvg)/WHEEL_COUNTS_PER_INCH/wheelGap);
        double robotX = (robot.armPortI.getCurrentPosition() / WHEEL_COUNTS_PER_INCH) - (2.5 * robotTheta);*/
        int POW = (robot.bpd.getCurrentPosition());
        int BOW = -robot.fpd.getCurrentPosition();
        int SOW = robot.bsd.getCurrentPosition();
        double robotY = ((POW+SOW)/2.0)/ODO_COUNTS_PER_INCH;

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
