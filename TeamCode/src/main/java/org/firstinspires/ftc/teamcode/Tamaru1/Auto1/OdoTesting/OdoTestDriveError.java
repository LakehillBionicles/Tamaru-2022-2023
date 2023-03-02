package org.firstinspires.ftc.teamcode.Tamaru1.Auto1.OdoTesting;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Tamaru1.Auto1.AutoBase;

////////////////////////////GOOD DRIVE//////////////////////////////
//import kotlin.ranges.URangesKt;   this caused an error
@Autonomous
@Disabled

public class OdoTestDriveError extends AutoBase {


    public void runOpMode() {
        super.runOpMode();
        startUp();
        waitForStart();

        if (opModeIsActive()) {

            robot.BOW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            telemetry.addData("current pos:", robot.BOW.getCurrentPosition());
            telemetry.update();

            sleep(1000);

            driveError(24, .20, .01);

            sleep(300);

            driveError(-24, .20, .01);

            sleep(50000);



        }
    }


    public void driveError(double targetDistance, double targetPower, double tolerance) {  //if want to go backwards, make dist and power negative

        robot.POW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.SOW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        newSPOWTarget = (((robot.POW.getCurrentPosition() + robot.SOW.getCurrentPosition()) / 2)/ODO_COUNTS_PER_INCH + (double) (targetDistance));//convert target dist l8r
        SPOWlocation = ((robot.POW.getCurrentPosition() + robot.SOW.getCurrentPosition()) / 2) /ODO_COUNTS_PER_INCH;
        driveError = (newSPOWTarget - SPOWlocation);

        while ((Math.abs(driveError)) > tolerance) {
            telemetry.addData("are we there yet?", "no");
            telemetry.addData("newSPOWTarget:", newSPOWTarget);
            telemetry.addData("SPOW location: ", SPOWlocation);
            telemetry.addData("drive error:", driveError);

            telemetry.update();

            robot.fpd.setPower(Math.signum(driveError) * targetPower);
            robot.bpd.setPower(Math.signum(driveError) * targetPower);
            robot.fsd.setPower(Math.signum(driveError) * targetPower);
            robot.bsd.setPower(Math.signum(driveError) * targetPower);

            SPOWlocation = ((robot.POW.getCurrentPosition() + robot.SOW.getCurrentPosition()) / 2) /ODO_COUNTS_PER_INCH;
            driveError = (newSPOWTarget - SPOWlocation);
            //updates the encoder's position inside the loop
        }
        telemetry.addData("are we there yet?", "yes");
        telemetry.addData("newSPOWTarget:", newSPOWTarget);
        telemetry.addData("SPOW location: ", SPOWlocation);
        telemetry.addData("drive error:", driveError);

        telemetry.update();

        robot.fpd.setPower(0);
        robot.bpd.setPower(0);
        robot.fsd.setPower(0);
        robot.bsd.setPower(0);


    }
}

