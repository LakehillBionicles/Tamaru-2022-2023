package org.firstinspires.ftc.teamcode.Tamaru2.Auto2.OldPaths;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Tamaru2.BaseClasses.Tamaru2Hardware;
import org.firstinspires.ftc.teamcode.Tamaru2.BaseClasses.AutoBase2;
/**
 * Class name: BlueCornerSinParking
 * Class Type: auto
 * Class Function: park in blue corner
 * Other Notes: uses sinDrive function and color sensor to park in blue corner
 */

@Autonomous
@Disabled

public class BlueCornerSinParking extends AutoBase2{
    Tamaru2Hardware robot = new Tamaru2Hardware();

    public String color = "";

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
        sinDrive(0,19,0,.35,.75,.35,.5,2,2);

        if(senseColorsFront().equals("green")){
            color = "green";
            telemetry.addData("color", color);
            telemetry.update();
        }else if(senseColorsFront().equals("blue")){
            color = "blue";
            telemetry.addData("color", color);
            telemetry.update();
        }else if(senseColorsFront().equals("red")){
            color = "red";
            telemetry.addData("color", color);
            telemetry.update();
        }


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

        }

        robot.fpd.setPower(0);
        robot.bpd.setPower(0);
        robot.fsd.setPower(0);
        robot.bsd.setPower(0);
    }

    public String senseColorsFront () {
        String colorStar = "blank";

        while (opModeIsActive() && colorStar.equals("blank")) {
            if (robot.colorSensorFront.red() > (robot.colorSensorFront.blue()) && robot.colorSensorFront.red() > (robot.colorSensorFront.green())) {
                colorStar = "red";
                telemetry.addData("i see red", " ");
                telemetry.update();
                colorStar = "red";
                //sleeveColor.equals(red);

            } else if (robot.colorSensorFront.blue() > (robot.colorSensorFront.red()) && robot.colorSensorFront.blue() > (robot.colorSensorFront.green())) {
                colorStar = "blue";
                telemetry.addData("i see blue", " ");
                telemetry.update();
                colorStar = "blue";

            } else if (robot.colorSensorFront.green() > (robot.colorSensorFront.red()) && robot.colorSensorFront.green() > (robot.colorSensorFront.blue())) {
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

    public String senseColorsPort(){
        String colorPort = "blank";

        while (opModeIsActive()&& colorPort.equals("blank")){

            if (robot.colorSensorPort.red() > (robot.colorSensorPort.blue()) && robot.colorSensorPort.red() > (robot.colorSensorPort.green())){
                colorPort = "red";
                telemetry.addData("i see red", " ");
                telemetry.update();
                colorPort ="red";


            } else if (robot.colorSensorPort.blue() > (robot.colorSensorPort.red()) && robot.colorSensorPort.blue() > (robot.colorSensorPort.green())){
                colorPort = "blue";
                telemetry.addData("i see blue", " ");
                telemetry.update();
                colorPort ="blue";

            } else if (robot.colorSensorPort.green() > (robot.colorSensorPort.red()) && robot.colorSensorPort.green() > (robot.colorSensorPort.blue())){
                colorPort = "green";
                telemetry.addData("i see green", " ");
                telemetry.update();
                colorPort = "green";

            }else {
                telemetry.addData("i see nothing", " ");
                telemetry.update();
                colorPort = "no go";
            }
        }

        return colorPort;
    }




}