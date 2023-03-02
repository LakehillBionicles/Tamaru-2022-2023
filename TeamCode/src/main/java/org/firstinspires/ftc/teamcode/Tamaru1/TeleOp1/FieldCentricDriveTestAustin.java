package org.firstinspires.ftc.teamcode.Tamaru1.TeleOp1;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.teamcode.Tamaru1.TemaruHardware;

@TeleOp
@Disabled

public class FieldCentricDriveTestAustin extends LinearOpMode{

    //this allows the heading to change in relation to the driver DOES NOT DRIVE STRAIGHT

    TemaruHardware robot = new TemaruHardware();
    public double SOWlocation = 0;
    public double POWlocation = 0;
    static final double COUNTS_PER_ODO_REV = 8192;
    static final double ODO_GEAR_REDUCTION = (1.0); // This is < 1.0 if geared UP
    static final double ODO_WHEEL_DIAMETER_INCHES = 2.0;  // For figuring circumference
    static final double ODO_COUNTS_PER_INCH = ((COUNTS_PER_ODO_REV * ODO_GEAR_REDUCTION) /
            (ODO_WHEEL_DIAMETER_INCHES * 3.1415));
    public double robotTheta = (-(((POWlocation - SOWlocation) / ODO_COUNTS_PER_INCH) / odoWheelGap));
    static final double odoWheelGap = 11.5;
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        robot.POW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.SOW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("robotTheta:", robotTheta);
        telemetry.update();
        robot.fpd.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.bpd.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.fsd.setDirection(DcMotorSimple.Direction.REVERSE); //reverse in code
        robot.bsd.setDirection(DcMotorSimple.Direction.FORWARD);
        //int hardware map

        waitForStart();

        if (isStopRequested()) return;

        telemetry.addData("does this work", "yes it does");
        telemetry.update();


        while (opModeIsActive()){
            POWlocation = robot.POW.getCurrentPosition();
            SOWlocation = robot.SOW.getCurrentPosition();
            robotTheta = (-(((POWlocation - SOWlocation) / ODO_COUNTS_PER_INCH) / odoWheelGap));

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double botHeading = -robotTheta;

            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

            double denominatorFCD = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rotX), 1);
            double fpder = (rotY + rotX + rx) / denominatorFCD;
            double bpdPower = (rotY - rotX + rx) / denominatorFCD;
            double fsdPower = (rotY - rotX - rx) / denominatorFCD;
            double bsdPower = (rotY + rotX - rx) / denominatorFCD;

            robot.fpd.setPower(fpder);
            robot.bpd.setPower(bpdPower);
            robot.fsd.setPower(fsdPower);
            robot.bsd.setPower(bsdPower);
            telemetry.addData("Sow positions", SOWlocation);
            telemetry.addData("Pow positions", POWlocation);
            telemetry.addData("Theta", robotTheta);
            telemetry.update();
        }


    }


}
