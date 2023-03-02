package org.firstinspires.ftc.teamcode.Tamaru2.Auto2.OldPaths;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Tamaru2.BaseClasses.Tamaru2Hardware;
import org.firstinspires.ftc.teamcode.Tamaru2.BaseClasses.AutoBase2;


@Autonomous
@Disabled

public class SinControlDrive extends AutoBase2{
    Tamaru2Hardware robot = new Tamaru2Hardware();

    public void runOpMode(){

        robot.init(hardwareMap);

        waitForStart();

        robot.fpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.fsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.fpd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.bpd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.fsd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.bsd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //positive is forward, port, ?
        sinDrive(0, -18, 0, .5, 1, .5, 2, 2, 1);
        sinDrive(3, 0, 0, .5, 1, .5, 2, 2, 2);
        stop();

    }

    public void sinDrive(double yTarget, double xTarget, double thetaTarget, double rotPower, double xPower, double yPower, double rotTol, double xTol, double yTol){
        robot.fpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.fsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.fpd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.bpd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.fsd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.bsd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        POWlocation = robot.fpd.getCurrentPosition();
        SOWlocation = robot.fsd.getCurrentPosition();
        BOWlocation = robot.bpd.getCurrentPosition();

        robotTheta = ((((POWlocation - SOWlocation) / ODO_COUNTS_PER_INCH) / odoWheelGap)); //CHECK THAT NEGATIVE NOT SURE
        robotX = (BOWlocation / ODO_COUNTS_PER_INCH) - (2.5 * robotTheta);
        robotY = (((robot.fpd.getCurrentPosition() + robot.fsd.getCurrentPosition()) / 2) /ODO_COUNTS_PER_INCH);

        newTargetTheta = Math.toRadians(thetaTarget) + robotTheta;
        newTargetX = ((xTarget) + robotX);
        newTargetY = ((yTarget) + robotY);

        xError = (newTargetX - robotX);
        yError = (newTargetY - robotY);
        thetaError = (newTargetTheta - robotTheta);

        double thetaPowerFunction = 0;
        double yPowerFunction = 0;
        double xPowerFunction = 0;

        while (Math.abs(thetaError) > Math.toRadians(rotTol) || Math.abs(xError) > xTol || Math.abs(yError) > yTol){


            odoTime = getRuntime();

            POWlocation = robot.fpd.getCurrentPosition();
            SOWlocation = robot.fsd.getCurrentPosition();
            BOWlocation = robot.bpd.getCurrentPosition();

            robotTheta = ((((POWlocation - SOWlocation) / ODO_COUNTS_PER_INCH) / odoWheelGap));
            robotX = (BOWlocation / ODO_COUNTS_PER_INCH) - (2.5 * robotTheta);
            robotY = (((POWlocation + SOWlocation) / 2) /ODO_COUNTS_PER_INCH);

            xError = (newTargetX - robotX);
            yError = (newTargetY - robotY);
            thetaError = (newTargetTheta - robotTheta);

            denominator = Math.max(Math.abs(yPower) + Math.abs(xPower) + Math.abs(thetaPower), 1);

            robot.fpd.setPower((yPowerFunction * yPower) + (-xPowerFunction * xPower) + (thetaPowerFunction * rotPower) / denominator);
            robot.bpd.setPower((yPowerFunction * yPower) - (-xPowerFunction * xPower) + (thetaPowerFunction * rotPower) / denominator);
            robot.fsd.setPower((yPowerFunction * yPower) - (-xPowerFunction * xPower) - (thetaPowerFunction * rotPower) / denominator);
            robot.bsd.setPower((yPowerFunction * yPower) + (-xPowerFunction * xPower) - (thetaPowerFunction * rotPower) / denominator);

            if(Math.abs(thetaError)>Math.toRadians(rotTol)){
                thetaPowerFunction = (-1*Math.signum(thetaError)*Math.abs(Math.sin(-1*thetaError )));
            }

            if(Math.abs(xError)>xTol){
                xPowerFunction = (-1*Math.signum(xError)*Math.abs(Math.sin(3.14/2 * Math.sin(3.14/2 * (-xError / (Math.max(Math.abs(newTargetX), 1.0)))))));
            }

            if(Math.abs(yError)>yTol){
                yPowerFunction = (-1*Math.signum(yError)*Math.abs(Math.sin(3.14/2 * Math.sin(3.14/2 * (-yError / (Math.max(Math.abs(newTargetY), 1.0)))))));
            }

            telemetry.addData("theta Power", thetaPowerFunction);
            telemetry.addData("x Power", xPowerFunction);
            telemetry.addData("y Power", yPowerFunction);
            telemetry.addData("robotTheta", robotTheta);
            telemetry.addData("robot x", robotX);
            telemetry.addData("robot y", robotY);
            telemetry.addData("theta error", thetaError);
            telemetry.addData("x error", xError);
            telemetry.addData("y error", yError);
            telemetry.addData("fpd Power", robot.fpd.getPower());
            telemetry.addData("bpd Power", robot.bpd.getPower());
            telemetry.addData("fsd Power", robot.fsd.getPower());
            telemetry.addData("bsd Power", robot.bsd.getPower());
            telemetry.addData("new target y", newTargetY);
            telemetry.update();
        }

        robot.fpd.setPower(0);
        robot.bpd.setPower(0);
        robot.fsd.setPower(0);
        robot.bsd.setPower(0);


    }
}