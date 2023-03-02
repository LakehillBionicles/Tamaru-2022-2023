package org.firstinspires.ftc.teamcode.Tamaru1.Auto1.OdoTesting;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Tamaru1.Auto1.AutoBase;


//import kotlin.ranges.URangesKt;   this caused an error
@Autonomous
@Disabled
//////////////////////////////GOOD THETA//////////////////////////////////
public class OdoThetaTestError extends AutoBase {


    public void runOpMode(){
        super.runOpMode();
        startUp();
        waitForStart();

        if (opModeIsActive()){

            robot.POW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.SOW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.BOW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);




            thetaTurnError(90, .20, 3);
            sleep(3000);
            thetaTurnError(-90, .20, 3);
            sleep(50000);






            //stop();

        }
    }




    public void thetaTurnError(double degrees, double targetPower, double tolerance){

        robot.POW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.SOW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BOW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        POWlocation = (robot.POW.getCurrentPosition());
        SOWlocation = (robot.SOW.getCurrentPosition());
        newTargetTheta = (Math.toRadians(degrees) - (((POWlocation - SOWlocation) / ODO_COUNTS_PER_INCH) / odoWheelGap));
        oldRobotTheta = -(((POWlocation - SOWlocation) / ODO_COUNTS_PER_INCH) / odoWheelGap);
        oldThetaError = (newTargetTheta - oldRobotTheta);
        while ((Math.abs(oldThetaError)) > Math.toRadians(tolerance)) {
            //POWlocation = (robot.POW.getCurrentPosition());
            //SOWlocation = (robot.SOW.getCurrentPosition());
            telemetry.addData("are we theta yet?", "no");
            telemetry.addData("newTargetTheta:", newTargetTheta);
            telemetry.addData("robot Theta: ", oldRobotTheta);
            telemetry.addData("theta error:", oldThetaError);
            telemetry.update();

            robot.fpd.setPower(-Math.signum(oldThetaError) * targetPower);
            robot.bpd.setPower(-Math.signum(oldThetaError) * targetPower);
            robot.fsd.setPower(Math.signum(oldThetaError) * targetPower);
            robot.bsd.setPower(Math.signum(oldThetaError) * targetPower);

            POWlocation = (robot.POW.getCurrentPosition());
            SOWlocation = (robot.SOW.getCurrentPosition());
            oldRobotTheta = -(((POWlocation - SOWlocation) / ODO_COUNTS_PER_INCH) / odoWheelGap); //updates the encoder's position inside the loop
            oldThetaError = (newTargetTheta - oldRobotTheta);
        }
        telemetry.addData("are we theta yet?", "yes");
        telemetry.addData("newTargetTheta:", newTargetTheta);
        telemetry.addData("robot Theta: ", oldRobotTheta);
        telemetry.addData("theta error:", oldThetaError);
        telemetry.update();
        robot.fpd.setPower(0);
        robot.bpd.setPower(0);
        robot.fsd.setPower(0);
        robot.bsd.setPower(0);

    }




}
