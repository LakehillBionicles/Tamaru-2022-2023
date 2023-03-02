package org.firstinspires.ftc.teamcode.Tamaru2.Auto2.OldPaths;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Tamaru2.BaseClasses.Tamaru2Hardware;
/**
 * Class name: PIDMethodTest
 * Class Type: auto
 * Class Function: test a PID loop we found on youtube
 * Other Notes: we never actually used this one, found something else that works well
 */
@Autonomous
@Disabled
public class PIDMethodTest extends LinearOpMode {
    Tamaru2Hardware robot = new Tamaru2Hardware();

    double integralSum = 0;
    double Kp = .33;
    double Ki = .33;
    double Kd = .33;

    public final double COUNTS_PER_ODO_REV = 8192;
    public final double ODO_GEAR_REDUCTION = (1.0); // This is < 1.0 if geared UP
    public final double ODO_WHEEL_DIAMETER_INCHES = 2.0;  // For figuring circumference
    public final double ODO_COUNTS_PER_INCH = ((COUNTS_PER_ODO_REV * ODO_GEAR_REDUCTION) /
            (ODO_WHEEL_DIAMETER_INCHES * 3.1415));

    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();

        robot.fpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.fsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.fpd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.bpd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.fsd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.bsd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive()) {
            robot.fpd.setPower(PIDControl(24, robot.fpd.getCurrentPosition()));
            robot.bpd.setPower(PIDControl(24, robot.bpd.getCurrentPosition()));
            robot.fsd.setPower(PIDControl(24, robot.fsd.getCurrentPosition()));
            robot.bsd.setPower(PIDControl(24, robot.bsd.getCurrentPosition()));
        }
    }

    public double PIDControl(double target, double state) {
        double reference = target/ODO_COUNTS_PER_INCH;
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError)/timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error*Kp) + (derivative*Kd) + (integralSum*Ki);
        return output;
    }
}
