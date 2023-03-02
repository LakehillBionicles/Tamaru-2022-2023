package org.firstinspires.ftc.teamcode.Tamaru2.Auto2.OldPaths;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Tamaru2.BaseClasses.Tamaru2Hardware;
import org.firstinspires.ftc.teamcode.Tamaru2.BaseClasses.AutoBase2;
/**
 * Class name: AutoPathing
 * Class Type: auto
 * Class Function: uses sinDrive function to score on low pole
 * Other Notes: doesn't work, sinDrive is a bad method
 */

@Autonomous
@Disabled
public class AutoPathing extends AutoBase2{
    Tamaru2Hardware robot = new Tamaru2Hardware();

    public String color = "";

    ////////////////////////////////TURRET TARGETS////////////////////////////////////////////////
    public static final double turretForward = .5;//not checked
    public static final double turretPort = 0;//not checked
    public static final double turretStar = 1;//not checked

    ////////////////////////////////HAND TARGETS////////////////////////////////////////////////
    public static final double handOpen = .8;
    public static final double handClosed = .5;

    ////////////////////////////////ARM TARGETS////////////////////////////////////////////////
    public final int downArmTarget = 0;
    public final int lowArmTargetPort = 2500;
    public final int midArmTargetPort = 4200;
    public final int highArmTargetPort = 6100;

    public final int lowArmTargetStar = 2700;
    public final int midArmTargetStar = 4400;
    public final int highArmTargetStar = 6300;

    public double portArmPos;
    public double starArmPos;


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

        robot.armPort.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armStar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //(yTarget, xTarget, thetaTarget, rotPower, xPower, yPower, rotTol, xTol, yTol)


        robot.servoHand.setPosition(robot.handClosed);
        sleep(1000);

        portArmPos = robot.armPort.getCurrentPosition();
        starArmPos = robot.armStar.getCurrentPosition();


        while((portArmPos<lowArmTargetPort || starArmPos<lowArmTargetStar) && opModeIsActive()){
            portArmPos = robot.armPort.getCurrentPosition();
            starArmPos = robot.armStar.getCurrentPosition();

            robot.armPort.setTargetPosition(highArmTargetPort);
            robot.armStar.setTargetPosition(highArmTargetStar);
            robot.armPort.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.armStar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.armPort.setPower(1);
            robot.armStar.setPower(1);

            telemetry.addData("status", "in loop");
            telemetry.update();
        }

        telemetry.addData("status", "out of loop");
        telemetry.update();

        sinDrive(0, -19, 0, .35, .75, .35, .5, 2, 2, 3);

        if(senseColorsStar().equals("green")){
            color = "green";
            telemetry.addData("color", color);
            telemetry.update();
        }else if(senseColorsStar().equals("blue")){
            color = "blue";
            telemetry.addData("color", color);
            telemetry.update();
        }else if(senseColorsStar().equals("red")){
            color = "red";
            telemetry.addData("color", color);
            telemetry.update();
        }

        //arm low pole: port = 3208, star = 3447


        sinDrive(0, -22, 0, .35, .75, .35, .5, 2, 2, 2);//in while loop?

        robot.servoHand.setPosition(robot.handOpen);

    /*if (color.equals("green")){
        sinDrive(0, 6, 0, .35, .75, .35, .5, 2, 2, 2);
        sinDrive(-22, 0, 0, .35, .35, .75, .5, 2, 2, 2);
    }
    else if (color.equals("blue")){
        telemetry.addData("parked", "blue");
        telemetry.update();
    }
    else{
    sinDrive(0, 6, 0, .35, .75, .35, .5, 2, 2, 2);
    sinDrive(22, 0, 0, .35, .35, .75, .5, 2, 2, 2);
    }*/

    }

    public void sinDrive(double yTarget, double xTarget, double thetaTarget, double rotPower, double xPower, double yPower, double rotTol, double xTol, double yTol, double timeout){
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

        resetRuntime();

        while ((Math.abs(thetaError) > Math.toRadians(rotTol) || Math.abs(xError) > xTol || Math.abs(yError) > yTol) && getRuntime()<timeout){

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

        }

        robot.fpd.setPower(0);
        robot.bpd.setPower(0);
        robot.fsd.setPower(0);
        robot.bsd.setPower(0);
    }

    public String senseColorsStar () {
        String colorStar = "blank";

        while (opModeIsActive() && colorStar.equals("blank")) {
            if (robot.colorSensorStar.red() > (robot.colorSensorStar.blue()) && robot.colorSensorStar.red() > (robot.colorSensorStar.green())) {
                colorStar = "red";
                telemetry.addData("i see red", " ");
                telemetry.update();
                colorStar = "red";
                //sleeveColor.equals(red);

            } else if (robot.colorSensorStar.blue() > (robot.colorSensorStar.red()) && robot.colorSensorStar.blue() > (robot.colorSensorStar.green())) {
                colorStar = "blue";
                telemetry.addData("i see blue", " ");
                telemetry.update();
                colorStar = "blue";

            } else if (robot.colorSensorStar.green() > (robot.colorSensorStar.red()) && robot.colorSensorStar.green() > (robot.colorSensorStar.blue())) {
                colorStar = "green";
                telemetry.addData("i see green", " ");
                telemetry.update();
                colorStar = "green";

            } else {
                telemetry.addData("i see nothing", " ");
                telemetry.update();
                colorStar = "no go";

            }

        }
        return colorStar;
    }
}