package org.firstinspires.ftc.teamcode.Tamaru1.TeleOp1;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.teamcode.Tamaru1.TemaruHardware;

/*@TeleOp
@Disabled

public class FieldCentricDriveTest extends LinearOpMode{

    //this allows the heading to change in relation to the driver DOES NOT DRIVE STRAIGHT

    TemaruHardware robot = new TemaruHardware();


    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);



        robot.fpd.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.bpd.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.fsd.setDirection(DcMotorSimple.Direction.REVERSE); //reverse in code
        robot.bsd.setDirection(DcMotorSimple.Direction.FORWARD);

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        //int hardware map

        waitForStart();

        if (isStopRequested()) return;

        telemetry.addData("does this work", "yes it does");
        telemetry.update();


        while (opModeIsActive()){

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double botHeading = -imu.getAngularOrientation().firstAngle;

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

        }


    }


}*/
