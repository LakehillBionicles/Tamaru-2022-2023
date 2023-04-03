package org.firstinspires.ftc.teamcode.Tamaru4;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Tamaru3.Tamaru3Hardware;

@Config
public class AutoBase4 extends LinearOpMode {
    public Tamaru3Hardware robot = new Tamaru3Hardware();

    public final double WHEEL_COUNTS_PER_INCH = 22.48958*(20.0/12.0);

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
    }


    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            /*newLeftTarget = robot.fpd.getCurrentPosition() + (int) (leftInches * WHEEL_COUNTS_PER_INCH);
            newRightTarget = robot.bpd.getCurrentPosition() + (int) (rightInches * WHEEL_COUNTS_PER_INCH);
            newLeftTarget = robot.fsd.getCurrentPosition() + (int) (leftInches * WHEEL_COUNTS_PER_INCH);
            newRightTarget = robot.bsd.getCurrentPosition() + (int) (rightInches * WHEEL_COUNTS_PER_INCH);*/

            newLeftTarget = (robot.fpd.getCurrentPosition() + robot.bpd.getCurrentPosition())/2 + (int) (leftInches * WHEEL_COUNTS_PER_INCH);
            newRightTarget = (robot.fsd.getCurrentPosition() + robot.bsd.getCurrentPosition())/2 + (int) (rightInches * WHEEL_COUNTS_PER_INCH);

            robot.fpd.setTargetPosition(newLeftTarget);
            robot.bpd.setTargetPosition(newRightTarget);
            robot.fsd.setTargetPosition(newLeftTarget);
            robot.bsd.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.fpd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bpd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.fsd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bsd.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            resetRuntime();
            robot.fpd.setPower(Math.abs(speed));
            robot.bpd.setPower(Math.abs(speed));
            robot.fsd.setPower(Math.abs(speed));
            robot.bsd.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (getRuntime() < timeoutS) &&
                    (robot.fpd.isBusy() && robot.bpd.isBusy() && robot.fsd.isBusy() && robot.bsd.isBusy())) {
            }

            // Stop all motion;
            robot.fpd.setPower(0);
            robot.bpd.setPower(0);
            robot.fsd.setPower(0);
            robot.bsd.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.fpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.fsd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bsd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

    }

    public void resetDrive(){
        robot.fpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.fsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.fpd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.bpd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.fsd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.bsd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}