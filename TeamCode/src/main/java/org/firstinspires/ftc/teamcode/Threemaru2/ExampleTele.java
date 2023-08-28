package org.firstinspires.ftc.teamcode.Threemaru2;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp
public class ExampleTele extends LinearOpMode {
    ExampleHardwareMap robot = new ExampleHardwareMap();

    public void runOpMode() {
        robot.init(hardwareMap);
        double motorPower = 0;

        waitForStart();

        while (opModeIsActive()) {

            motorPower = -gamepad1.left_stick_y;
            robot.fpd.setPower(motorPower);

        }
    }
}
