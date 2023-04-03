package org.firstinspires.ftc.teamcode.Tamaru1.TeleOp1;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Tamaru1.TemaruHardware;
import org.firstinspires.ftc.teamcode.Tamaru1.Auto1.AutoBase;

@TeleOp
@Disabled
public class testlaksdnfl extends AutoBase {

    TemaruHardware robot = new TemaruHardware();


    public static double testDriveSpeed = 1.0;
    double speed = 1;
    double speedStrafe = 0.7;
    double startHeading;
    boolean isMoving;

    public ElapsedTime runtime = new ElapsedTime(); //might cause an error (not sure)


    public void runOpMode() {

        robot.init(hardwareMap);

        waitForStart();

        telemetry.addData("does this work", "yes it does");
        telemetry.update();


        while (opModeIsActive()) {
            driveFieldCentric(gamepad1.left_stick_x, gamepad1.left_stick_y);
        }
    }

    public void driveFieldCentric(double stickX, double stickY) {

        if (stickY < -.25 || stickY > .25 || stickX < -.25 || stickX > .25) {
            isMoving = true;
            double angle = Math.atan2(stickY, stickX);
            double magnitude = Math.sqrt(Math.pow(stickY, 2) + Math.pow(stickX, 2));
            robotTheta = (-(((POWlocation - SOWlocation) / ODO_COUNTS_PER_INCH) / odoWheelGap));
            double currentHeading = robotTheta;
            robot.fsd.setPower((Math.sin(currentHeading + angle + Math.PI / 4)) * magnitude * speed);
            robot.bpd.setPower((Math.sin(currentHeading + angle + Math.PI / 4)) * magnitude * speed);
            robot.fpd.setPower((Math.sin(currentHeading + angle - Math.PI / 4)) * magnitude * speed);
            robot.bsd.setPower((Math.sin(currentHeading + angle - Math.PI / 4)) * magnitude * speed);
        }
        else {
            robot.fpd.setPower(0.0);
            robot.bpd.setPower(0.0);
            robot.fsd.setPower(0.0);
            robot.bsd.setPower(0.0);
        }


    }

}


