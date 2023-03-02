package org.firstinspires.ftc.teamcode.Tamaru2.TeleOp2.PIDTesting;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Tamaru2.BaseClasses.Tamaru2Hardware;
import org.firstinspires.ftc.teamcode.Tamaru2.BaseClasses.AutoBase2;


@Autonomous
@Disabled

public class odoTesting extends AutoBase2 {
    Tamaru2Hardware robot = new Tamaru2Hardware();

    //////////////////////////////////// ODOMETRY WHEEL CALCULATIONS //////////////////////////////////////////////
    public final double COUNTS_PER_ODO_REV = 8192;
    public final double ODO_GEAR_REDUCTION = (1.0); // This is < 1.0 if geared UP
    public final double ODO_WHEEL_DIAMETER_INCHES = 2.0;  // For figuring circumference
    public final double ODO_COUNTS_PER_INCH = ((COUNTS_PER_ODO_REV * ODO_GEAR_REDUCTION) /
            (ODO_WHEEL_DIAMETER_INCHES * 3.1415));

    public final double odoWheelGap = 12.5;//used to be 11.5

    private double BOWlocation;
    private double POWlocation;
    private double SOWlocation;

    private double robotTheta;
    private double robotX;
    private double robotY;

    public double fpdPower;
    public double bpdPower;
    public double fsdPower;
    public double bsdPower;

    public double loops;

    public void runOpMode() {

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


        while (opModeIsActive()) {
            BOWlocation = robot.bpd.getCurrentPosition();
            POWlocation = robot.fpd.getCurrentPosition();
            SOWlocation = robot.fsd.getCurrentPosition();

            robotTheta = (-(((POWlocation - SOWlocation) / ODO_COUNTS_PER_INCH) / odoWheelGap));
            robotX = (BOWlocation / ODO_COUNTS_PER_INCH) - (2.5 * robotTheta);//changed from 5 to 2.5
            robotY = (((POWlocation + SOWlocation) / 2) / ODO_COUNTS_PER_INCH);

            testCoordinateDrive(0, 0, 24, .1, .1, .1, 2, .25, .25);
        }
    }

    public void testCoordinateDrive(double thetaTarget, double xTarget, double yTarget, double rotPower, double linXPower, double linYPower, double rotTol, double xTol, double yTol){
        POWlocation = robot.fpd.getCurrentPosition();
        SOWlocation = robot.fsd.getCurrentPosition();
        BOWlocation = robot.bpd.getCurrentPosition();

        robotTheta = (-(((POWlocation - SOWlocation) / ODO_COUNTS_PER_INCH) / odoWheelGap));
        robotX = (BOWlocation / ODO_COUNTS_PER_INCH)- (2.5 * robotTheta);//changed from 5 to 2.5
        robotY = (((POWlocation + SOWlocation) / 2) /ODO_COUNTS_PER_INCH);

        newTargetTheta = Math.toRadians(thetaTarget) + robotTheta;
        newTargetX = ((xTarget) + robotX);
        newTargetY = ((yTarget) + robotY);

        xError = (newTargetX - robotX);
        yError = (newTargetY - robotY);
        thetaError = (newTargetTheta - robotTheta);

        while (Math.abs(thetaError) > Math.toRadians(rotTol) || Math.abs(xError) > xTol || Math.abs(yError) > yTol){

            odoTime = getRuntime();

            POWlocation = robot.fpd.getCurrentPosition();
            SOWlocation = robot.fsd.getCurrentPosition();
            BOWlocation = robot.bpd.getCurrentPosition();

            robotTheta = (-(((POWlocation - SOWlocation) / ODO_COUNTS_PER_INCH) / odoWheelGap));
            robotX = (BOWlocation / ODO_COUNTS_PER_INCH) - (2.5 * robotTheta);
            robotY = (((POWlocation + SOWlocation) / 2) / ODO_COUNTS_PER_INCH);

            xError = (newTargetX - robotX);
            yError = (newTargetY - robotY);
            thetaError = (newTargetTheta - robotTheta);

            denominator = Math.max(Math.abs(yPower) + Math.abs(xPower) + Math.abs(thetaPower), 1);

            if(loops<5){
                fpdPower = Math.signum(fpdPower)*.9;
                bpdPower = Math.signum(bpdPower)*.9;
                fsdPower = Math.signum(fsdPower)*.9*.7;
                bsdPower = Math.signum(bsdPower)*.9;
                telemetry.addData("fpdPower", fpdPower);
                telemetry.update();
            }

            //made yPower negative
            robot.fpd.setPower(((linYPower * (-yPower)) + (linXPower * (xPower)) + (rotPower * thetaPower))/denominator);
            robot.bpd.setPower(((linYPower * (-yPower)) - (linXPower * (xPower)) + (rotPower * thetaPower))/denominator);
            robot.fsd.setPower(((linYPower * (-yPower)) - (linXPower * (xPower)) - (rotPower * thetaPower))/denominator);
            robot.bsd.setPower(((linYPower * (-yPower)) + (linXPower * (xPower)) - (rotPower * thetaPower))/denominator);

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
            } else{
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
            } else{
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
            } else{
                yPower = 0;
            }

            loops++;

        }

        robot.fpd.setPower(0);
        robot.bpd.setPower(0);
        robot.fsd.setPower(0);
        robot.bsd.setPower(0);

    }
}