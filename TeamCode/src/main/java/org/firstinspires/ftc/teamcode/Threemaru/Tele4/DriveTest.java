package org.firstinspires.ftc.teamcode.Threemaru.Tele4;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Threemaru.ThreemaruHardware;

//@Disabled
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
            double fpd = robot.fpd.getCurrentPosition();
            double bpd = robot.bpd.getCurrentPosition();
            double fsd = robot.fsd.getCurrentPosition();

            telemetry.addData("SOW", fsd);//FORWARD
            telemetry.addData("POW", bpd);//REVERSE
            telemetry.addData("BOW", fpd);//FORWARD
            telemetry.update();
        }
    }
}