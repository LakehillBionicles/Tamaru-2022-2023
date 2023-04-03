package org.firstinspires.ftc.teamcode.Tamaru2.TeleOp2.Old;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Tamaru2.BaseClasses.Tamaru2Hardware;

@TeleOp
@Disabled

public class testMotors extends LinearOpMode {
    Tamaru2Hardware robot = new Tamaru2Hardware();

    public void runOpMode() {

        robot.init(hardwareMap);

        waitForStart();


        while (opModeIsActive()) {

            if(gamepad1.left_stick_y<0 || gamepad1.left_stick_y>0) {
                robot.fpd.setPower(gamepad1.left_stick_y);
                robot.bpd.setPower(gamepad1.left_stick_y);
                robot.fsd.setPower(gamepad1.left_stick_y);
                robot.bsd.setPower(gamepad1.left_stick_y);
            }else if(gamepad1.left_stick_x<0 || gamepad1.left_stick_x>0){
                robot.fpd.setPower((-.9*gamepad1.left_stick_x));
                robot.bpd.setPower(.9*gamepad1.left_stick_x);
                robot.fsd.setPower(.9*gamepad1.left_stick_x);
                robot.bsd.setPower(-.9*gamepad1.left_stick_x);
            }else{
                robot.fpd.setPower(0);
                robot.bpd.setPower(0);
                robot.fsd.setPower(0);
                robot.bsd.setPower(0);
            }

        }

    }
}