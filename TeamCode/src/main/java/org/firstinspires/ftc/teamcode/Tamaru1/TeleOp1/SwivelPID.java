package org.firstinspires.ftc.teamcode.Tamaru1.TeleOp1;

import static org.firstinspires.ftc.teamcode.Tamaru1.TemaruHardware.armSpeed;
import static org.firstinspires.ftc.teamcode.Tamaru1.TemaruHardware.closeHandPos;
import static org.firstinspires.ftc.teamcode.Tamaru1.TemaruHardware.openHandPos;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Tamaru1.TemaruHardware;
import org.firstinspires.ftc.teamcode.Tamaru1.Auto1.AutoBase;

@TeleOp
//@Disabled

//////////////////////gamepad1 is drive; gamepad 2 is arm/hand/pre-set distances//////////////////////

public class SwivelPID extends LinearOpMode {
    TemaruHardware robot = new TemaruHardware();

    private double power;
    private double location;
    private double target = 0;
    private double error;
    private double time;
    private double denominator;
    private double integralSumLimit = .25;
    private double integralSum;
    private double derivative;
    private double lastError;
    private double maxPower = 0.25;

    private double Kp = .2;
    private double Ki = .7;
    private double Kd = .1;

    public ElapsedTime runtime = new ElapsedTime();


    public void runOpMode() {

        robot.init(hardwareMap);

        waitForStart();

        telemetry.addData("update date", "____");
        telemetry.update();

        robot.arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        while (opModeIsActive()) {
            if(gamepad2.dpad_left) {
                target = 67;
            } else if(gamepad2.dpad_down) {
                target = 141;
            } else if(gamepad2.dpad_right) {
                target = -50;
            } else if(gamepad2.dpad_up) {
                target = 0;
            }

            location = robot.arm2.getCurrentPosition();
            error = (target - location);
            time = getRuntime();
            denominator = Math.max(Math.abs(power), .25);

            robot.fpd.setPower(maxPower * power/denominator);

            derivative = ((error - lastError) / time);
            integralSum = (integralSum + (error * time));
            if (integralSum > integralSumLimit){
                integralSum = integralSumLimit;
            }
            if (integralSum < -integralSumLimit){
                integralSum = -integralSumLimit;
            }
            power = -((Kd * derivative) + (Ki * integralSum) + (Kp * Math.signum(error)));
        }

        robot.arm2.setPower(power);

    }
}
