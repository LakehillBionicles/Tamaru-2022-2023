package org.firstinspires.ftc.teamcode.Tamaru3.TeleOp3.PIDTuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor; //DcMotorEx?

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Tamaru3.Tamaru3Hardware;

@Config
@TeleOp
public class PIDTuningTheta extends OpMode{
    Tamaru3Hardware robot = new Tamaru3Hardware();
    private PIDController controller;

    BNO055IMU imu;
    Orientation angles;

    public static double p = 0, i = 0, d = 0;

    public static int target = 0;



    /*public final double COUNTS_PER_ODO_REV = 8192;
    public final double ODO_GEAR_REDUCTION = (1.0); // This is < 1.0 if geared UP
    public final double ODO_WHEEL_DIAMETER_INCHES = 2.0;  // For figuring circumference
    public final double ODO_COUNTS_PER_INCH = ((COUNTS_PER_ODO_REV * ODO_GEAR_REDUCTION) /
            (ODO_WHEEL_DIAMETER_INCHES * 3.1415));

    public final double odoWheelGap = 12.5;*/

    /*public final double COUNTS_PER_WHEEL_REV = 28;//counts @ motor
     public final double WHEEL_GEAR_REDUCTION = (10.4329);
     public final double WHEEL_DIAMETER_INCHES = 3.779;  // For figuring circumference
     public final double WHEEL_COUNTS_PER_INCH = ((COUNTS_PER_WHEEL_REV * WHEEL_GEAR_REDUCTION) /
             (WHEEL_DIAMETER_INCHES * 3.1415));*/
    public final double WHEEL_COUNTS_PER_INCH = 22.48958;

    public final double wheelGap = 11.5;

    @Override
    public void init(){
        robot.init(hardwareMap);
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

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

        //double robotTheta = ((robot.armPort_POW.getCurrentPosition()-robot.SOW.getCurrentPosition())/ODO_COUNTS_PER_INCH / odoWheelGap);
        int fpdPos = robot.fpd.getCurrentPosition();
        int bpdPos = robot.bpd.getCurrentPosition();
        int fsdPos = robot.fsd.getCurrentPosition();
        int bsdPos = robot.bsd.getCurrentPosition();
        double robotY = ((fpdPos+bpdPos+fsdPos+bsdPos)/4.0)/WHEEL_COUNTS_PER_INCH;
        double robotTheta = (((fsdPos-fpdPos)/WHEEL_COUNTS_PER_INCH/wheelGap));
        double robotX = (robot.armPortI.getCurrentPosition() / WHEEL_COUNTS_PER_INCH) - (2.5 * robotTheta);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double pidTheta = controller.calculate(angles.firstAngle, target);

        double velocityTheta = -pidTheta * robot.maxVelocity;

        robot.fpd.setVelocity(velocityTheta);
        robot.bpd.setVelocity(velocityTheta);
        robot.fsd.setVelocity(-velocityTheta);
        robot.bsd.setVelocity(-velocityTheta);

        telemetry.addData("robotTheta", angles.firstAngle);
        telemetry.addData("target", target);
        telemetry.addData("velocityTheta", velocityTheta);
        telemetry.addData("fpd", robot.fpd.getCurrentPosition());
        telemetry.addData("bpd", robot.bpd.getCurrentPosition());
        telemetry.addData("fsd", robot.fsd.getCurrentPosition());
        telemetry.addData("bsd", robot.bsd.getCurrentPosition());
        telemetry.update();
    }
}
