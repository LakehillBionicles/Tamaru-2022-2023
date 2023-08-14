package org.firstinspires.ftc.teamcode.Threemaru2.ThreemaruTele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Threemaru2.Threemaru2Hardware;

@TeleOp
public class MotorDirections extends LinearOpMode {
    Threemaru2Hardware robot = new Threemaru2Hardware();
        public void runOpMode() {
            robot.init(hardwareMap);
            robot.bsd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.bpd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.fsd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.fpd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            waitForStart();
            while (opModeIsActive()) {
                    robot.fpd.setPower(.5);
                    robot.bpd.setPower(.5);
                    robot.fsd.setPower(.5);
                    robot.bsd.setPower(.5);
            }
    }
}