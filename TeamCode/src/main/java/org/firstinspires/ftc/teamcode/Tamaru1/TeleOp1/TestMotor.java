package org.firstinspires.ftc.teamcode.Tamaru1.TeleOp1;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Tamaru1.TemaruHardware;
import org.firstinspires.ftc.teamcode.Tamaru1.Auto1.AutoBase;

@TeleOp
@Disabled

public class TestMotor extends LinearOpMode {
    //TemaruHardware robot = new TemaruHardware();


    public DcMotor testMotor1 = null;
    public DcMotor testMotor2 = null;


    public static double testDriveSpeed = 1.0;
    double speed = 1;
    double speedStrafe = 0.7;
    double startHeading;
    boolean isMoving;

    public void runOpMode() {

        //robot.init(hardwareMap);


        //testMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        //testMotor2.setDirection(DcMotorSimple.Direction.FORWARD);

        //testMotor1.setPower(0);
        //testMotor2.setPower(0);

        //testMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //testMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        telemetry.addData("does this work", "yes it does");
        telemetry.update();


        while (opModeIsActive()) {


            if (gamepad1.left_trigger > 0) {

                testMotor1.setPower(0.5);
                testMotor2.setPower(-.5);

            } else if (gamepad1.right_trigger > 0) {

                testMotor1.setPower(-.5);
                testMotor2.setPower(.5);

            } else {

                testMotor1.setPower(0);
                testMotor2.setPower(0);

            }

        }

    }
}




















