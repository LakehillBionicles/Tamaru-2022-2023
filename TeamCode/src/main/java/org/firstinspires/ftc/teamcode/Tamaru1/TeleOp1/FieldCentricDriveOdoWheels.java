package org.firstinspires.ftc.teamcode.Tamaru1.TeleOp1;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Tamaru1.TemaruHardware;
import org.firstinspires.ftc.teamcode.Tamaru1.Auto1.AutoBase;

@TeleOp
@Disabled


public class FieldCentricDriveOdoWheels extends AutoBase {

    TemaruHardware robot = new TemaruHardware();

    public void runOpMode() {

        robot.init(hardwareMap);

        waitForStart();

        telemetry.addData("does this work", "yes it does");
        telemetry.update();

        robot.POW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.SOW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BOW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while (opModeIsActive()) {




            if (gamepad1.left_stick_y > 0.2){
                coordinateDrive(0,0,5,0.1,0.1,0.1,2,.2,.2);



            } else if (gamepad1.left_stick_y < -0.2) {
                coordinateDrive(0,0,5,0.1,0.1,0.1,2,.2,.2);

            } else {
                robot.fpd.setPower(0);
                robot.bpd.setPower(0);
                robot.fsd.setPower(0);
                robot.bsd.setPower(0);


            }










        }



    }

    static final double COUNTS_PER_ODO_REV = 8192;
    static final double ODO_GEAR_REDUCTION = (1.0); // This is < 1.0 if geared UP
    static final double ODO_WHEEL_DIAMETER_INCHES = 2.0;  // For figuring circumference
    static final double ODO_COUNTS_PER_INCH = ((COUNTS_PER_ODO_REV * ODO_GEAR_REDUCTION) /
            (ODO_WHEEL_DIAMETER_INCHES * 3.1415));

    static final double odoWheelGap = 11.5;

    public void autoWhereAmI(){
        POWlocation = robot.POW.getCurrentPosition();
        SOWlocation = robot.SOW.getCurrentPosition();
        BOWlocation = robot.BOW.getCurrentPosition();

        robotTheta = (-(((POWlocation - SOWlocation) / ODO_COUNTS_PER_INCH) / odoWheelGap));
        robotX = (BOWlocation / ODO_COUNTS_PER_INCH) - (5 * robotTheta);
        robotY = (((robot.POW.getCurrentPosition() + robot.SOW.getCurrentPosition()) / 2) /ODO_COUNTS_PER_INCH);

        xError = (newTargetX - robotX);
        yError = (newTargetY - robotY);
        thetaError = (newTargetTheta - robotTheta);

        telemetry.addData("robotTheta:", robotTheta);
        telemetry.addData("robotX:", robotX);
        telemetry.addData("robotY:", robotY);

        telemetry.addData("thetaError:", thetaError);
        telemetry.addData("xError:", xError);
        telemetry.addData("yError:", yError);

        telemetry.update();

    }


    public void coordinateDrive(double thetaTarget, double xTarget, double yTarget, double rotPower, double linXPower, double linYPower, double rotTol, double xTol, double yTol){
        POWlocation = robot.POW.getCurrentPosition();
        SOWlocation = robot.SOW.getCurrentPosition();
        BOWlocation = robot.BOW.getCurrentPosition();

        robotTheta = (-(((POWlocation - SOWlocation) / ODO_COUNTS_PER_INCH) / odoWheelGap));
        robotX = (BOWlocation / ODO_COUNTS_PER_INCH)- (5 * robotTheta);
        robotY = (((robot.POW.getCurrentPosition() + robot.SOW.getCurrentPosition()) / 2) /ODO_COUNTS_PER_INCH);

        newTargetTheta = Math.toRadians(thetaTarget) + robotTheta;
        newTargetX = ((xTarget) + robotX);
        newTargetY = ((yTarget) + robotY);

        xError = (newTargetX - robotX);
        yError = (newTargetY - robotY);
        thetaError = (newTargetTheta - robotTheta);

        telemetry.addData("robotTheta:", robotTheta);
        telemetry.addData("robotX:", robotX);
        telemetry.addData("robotY:", robotY);

        telemetry.addData("newTargetTheta:", newTargetTheta);
        telemetry.addData("newTargetX:", newTargetX);
        telemetry.addData("newTargetY:", newTargetY);

        telemetry.addData("thetaError:", thetaError);
        telemetry.addData("xError:", xError);
        telemetry.addData("yError:", yError);

        telemetry.addData("thetaPower:", thetaPower);
        telemetry.addData("xPower:", xPower);
        telemetry.addData("yPower:", yPower);

        telemetry.update();

        while (Math.abs(thetaError) > Math.toRadians(rotTol) || Math.abs(xError) > xTol || Math.abs(yError) > yTol){

            odoTime = getRuntime();

            autoWhereAmI();

            denominator2 = Math.max(Math.abs(yPower) + Math.abs(xPower) + Math.abs(thetaPower), 1);

            robot.fpd.setPower(((linYPower * (yPower)) + (linXPower * (xPower)) + (rotPower * thetaPower))/denominator);
            robot.bpd.setPower(((linYPower * (yPower)) - (linXPower * (xPower)) + (rotPower * thetaPower))/denominator);
            robot.fsd.setPower(((linYPower * (yPower)) - (linXPower * (xPower)) - (rotPower * thetaPower))/denominator);
            robot.bsd.setPower(((linYPower * (yPower)) + (linXPower * (xPower)) - (rotPower * thetaPower))/denominator);

            if (Math.abs(thetaError) > Math.toRadians(rotTol)){
                thetaDeriv = ((thetaError - lastThetaError) / odoTime);
                thetaIntegralSum = (thetaIntegralSum + (thetaError * odoTime));
                if (thetaIntegralSum > integralSumLimit){
                    thetaIntegralSum = integralSumLimit;
                }
                if (thetaIntegralSum < -integralSumLimit){
                    thetaIntegralSum = -integralSumLimit;
                }
                thetaPower = -((Kd * thetaDeriv) + (Ki * thetaIntegralSum) + (Kp * Math.signum(thetaError)));
            }
            else{
                thetaPower = 0;
            }

            if (Math.abs(xError) > xTol){
                xDeriv = ((xError - lastXError) / odoTime);
                xIntegralSum = (xIntegralSum + (xError * odoTime));
                if (xIntegralSum > integralSumLimit){
                    xIntegralSum = integralSumLimit;
                }
                if (xIntegralSum < -integralSumLimit){
                    xIntegralSum = -integralSumLimit;
                }
                xPower = ((Kd * xDeriv) + (Ki * xIntegralSum) + (Kp * Math.signum(xError)));
            }
            else{
                xPower = 0;
            }

            if (Math.abs(yError) > yTol){
                yDeriv = ((yError - lastYError) / odoTime);
                yIntegralSum = (yIntegralSum + (yError * odoTime));
                if (yIntegralSum > integralSumLimit){
                    yIntegralSum = integralSumLimit;
                }
                if (yIntegralSum < -integralSumLimit){
                    yIntegralSum = -integralSumLimit;
                }
                yPower = ((Kd * yDeriv) + (Ki * yIntegralSum) + (Kp * Math.signum(yError)));
            }
            else{
                yPower = 0;
            }

            telemetry.addData("robotTheta:", robotTheta);
            telemetry.addData("robotX:", robotX);
            telemetry.addData("robotY:", robotY);

            telemetry.addData("newTargetTheta:", newTargetTheta);
            telemetry.addData("newTargetX:", newTargetX);
            telemetry.addData("newTargetY:", newTargetY);

            telemetry.addData("thetaError:", thetaError);
            telemetry.addData("xError:", xError);
            telemetry.addData("yError:", yError);

            telemetry.addData("thetaPower:", thetaPower);
            telemetry.addData("xPower:", xPower);
            telemetry.addData("yPower:", yPower);

            telemetry.addData("runtime", getRuntime());

            telemetry.update();

            //motor use to be here
        }

        robot.fpd.setPower(0);
        robot.bpd.setPower(0);
        robot.fsd.setPower(0);
        robot.bsd.setPower(0);
        telemetry.addData("robotTheta:", robotTheta);
        telemetry.addData("robotX:", robotX);
        telemetry.addData("robotY:", robotY);

        telemetry.addData("newTargetTheta:", newTargetTheta);
        telemetry.addData("newTargetX:", newTargetX);
        telemetry.addData("newTargetY:", newTargetY);

        telemetry.addData("thetaError:", thetaError);
        telemetry.addData("xError:", xError);
        telemetry.addData("yError:", yError);

        telemetry.addData("thetaPower:", thetaPower);
        telemetry.addData("xPower:", xPower);
        telemetry.addData("yPower:", yPower);

        telemetry.addData("yay", "made it");
        telemetry.update();

    }

}
