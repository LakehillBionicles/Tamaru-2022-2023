package org.firstinspires.ftc.teamcode.Tamaru1.Auto1.OdoTesting;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Tamaru1.Auto1.AutoBase;


//import kotlin.ranges.URangesKt;   this caused an error
@Autonomous
@Disabled

////////////////////////////GOOD STRAFE/////////////////////////////////////////

public class OdoTestStrafeError extends AutoBase {


    public void runOpMode() {
        super.runOpMode();
        startUp();
        waitForStart();

        if (opModeIsActive()) {

            robot.BOW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            telemetry.addData("current pos:", robot.BOW.getCurrentPosition());
            telemetry.update();

            sleep(1000);

            strafeError(24, .20, .01, 50000);

            sleep(300);

            strafeError(-24, .20, .01, 50000);

            sleep(50000);


            /*encoderDrive(0.5, 24, 24, 20);

            sleep(1000);

                        telemetry.addData("right ticks: ", getRightTicks());
            telemetry.addData("left ticks: ", getLeftTicks());
            telemetry.addData("front ticks: ", getFrontTicks());
            telemetry.update();


            sleep(2000000); */


            //encoderDrive(0.5, 24, 24, 20);


            //stop();

        }
    }


    public void strafeError(double targetDistance, double targetPower, double tolerance, double timeoutS) {  //if want to go backwards, make dist and power negative

        robot.BOW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        newBOWTarget = ((robot.BOW.getCurrentPosition()) / ODO_COUNTS_PER_INCH + (double) (targetDistance));//convert target dist l8r
        BOWlocation = ((robot.BOW.getCurrentPosition()) / ODO_COUNTS_PER_INCH);
        strafeError = (newBOWTarget - BOWlocation);
        resetRuntime();
        while ((Math.abs(strafeError)) > tolerance) {
            telemetry.addData("are we there yet?", "no");
            telemetry.addData("newBOWTarget:", newBOWTarget);
            telemetry.addData("BOW location: ", BOWlocation);
            telemetry.addData("strafe error:", strafeError);
            telemetry.addData("avgEncoderPos:", robot.BOW.getCurrentPosition() / ODO_COUNTS_PER_INCH);

            telemetry.update();

            robot.fpd.setPower(Math.signum(strafeError) * targetPower);
            robot.bpd.setPower(-Math.signum(strafeError) * targetPower);
            robot.fsd.setPower(-Math.signum(strafeError) * targetPower);
            robot.bsd.setPower(Math.signum(strafeError) * targetPower);

            BOWlocation = (robot.BOW.getCurrentPosition() / ODO_COUNTS_PER_INCH);
            strafeError = (newBOWTarget - BOWlocation);
            //updates the encoder's position inside the loop
        }
        telemetry.addData("are we there yet?", "yes");
        telemetry.addData("newBOWTarget:", newBOWTarget);
        telemetry.addData("BOW location: ", BOWlocation);
        telemetry.addData("strafe error:", strafeError);
        telemetry.addData("avgEncoderPos:", robot.BOW.getCurrentPosition());

        robot.fpd.setPower(0);
        robot.bpd.setPower(0);
        robot.fsd.setPower(0);
        robot.bsd.setPower(0);


    }
}

