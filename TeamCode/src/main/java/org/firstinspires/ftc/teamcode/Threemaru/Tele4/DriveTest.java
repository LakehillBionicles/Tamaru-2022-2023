package org.firstinspires.ftc.teamcode.Threemaru.Tele4;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp
public class DriveTest extends LinearOpMode {
    ThreemaruHardware robot = new ThreemaruHardware();
    public void runOpMode() {
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
            robot.fpd.setPower(-gamepad1.left_stick_y/2 + gamepad1.right_stick_x/2);
            robot.bpd.setPower(-gamepad1.left_stick_y/2 + gamepad1.right_stick_x/2);
            robot.fsd.setPower(-gamepad1.left_stick_y/2 - gamepad1.right_stick_x/2);
            robot.bsd.setPower(-gamepad1.left_stick_y/2 - gamepad1.right_stick_x/2);
        }
    }
}