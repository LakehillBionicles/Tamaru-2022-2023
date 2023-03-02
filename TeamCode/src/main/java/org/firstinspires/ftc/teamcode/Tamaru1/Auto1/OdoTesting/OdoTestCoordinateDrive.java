package org.firstinspires.ftc.teamcode.Tamaru1.Auto1.OdoTesting;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Tamaru1.Auto1.AutoBase;

@Autonomous
@Disabled

//FOR NOTEBOOK: graph error vs power (to find perfect tolerance)


public class OdoTestCoordinateDrive extends AutoBase {
    public void runOpMode() {
        super.runOpMode();
        startUp();
        waitForStart();

        if (opModeIsActive()) {
            robot.BOW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.POW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.SOW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            coordinateDrive(0, 0, 24, .2, 0.2, 0.2, 2, .5, 0.5); //theta x y power power power tol tol tol

            sleep(5000);

            //coordinateDrive(-90, -24, -24, .2, 0.2, 0.2, 2, .5, 0.5);

            //y tolerance and power were 0.5 (delta y = 24)
            //theta tolerance 1? or 2? and power 0.2 (delta theta = 90)
            //x tolerance and power 0.5 (delta theta = 24)

            sleep(50000);
        }
    }

    public void whereAmI(){
        POWlocation = robot.POW.getCurrentPosition();
        SOWlocation = robot.SOW.getCurrentPosition();
        BOWlocation = robot.BOW.getCurrentPosition();

        robotTheta = (-(((POWlocation - SOWlocation) / ODO_COUNTS_PER_INCH) / odoWheelGap));
        robotX = (BOWlocation / ODO_COUNTS_PER_INCH);
        robotY = (((robot.POW.getCurrentPosition() + robot.SOW.getCurrentPosition()) / 2) /ODO_COUNTS_PER_INCH);

        telemetry.addData("robotTheta:", robotTheta);
        telemetry.addData("robotX:", robotX);
        telemetry.addData("robotY:", robotY);

        telemetry.update();
    }

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

        /*telemetry.addData("robotTheta:", robotTheta);
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

        telemetry.update();*/

        while (Math.abs(thetaError) > Math.toRadians(rotTol) || Math.abs(xError) > xTol || Math.abs(yError) > yTol){

            autoWhereAmI();


            robot.fpd.setPower(((linYPower * (yPower)) + (linXPower * (xPower)) + (rotPower * thetaPower))/denominator);
            robot.bpd.setPower(((linYPower * (yPower)) - (linXPower * (xPower)) + (rotPower * thetaPower))/denominator);
            robot.fsd.setPower(((linYPower * (yPower)) - (linXPower * (xPower)) - (rotPower * thetaPower))/denominator);
            robot.bsd.setPower(((linYPower * (yPower)) + (linXPower * (xPower)) - (rotPower * thetaPower))/denominator);

            if (Math.abs(thetaError) > Math.toRadians(rotTol)){
                thetaPower = -(Math.signum(thetaError));
            }
            else{
                thetaPower = 0;
            }

            if (Math.abs(xError) > xTol){
                xPower = Math.signum(xError);
            }
            else{
                xPower = 0;
            }

            if (Math.abs(yError) > yTol){
                yPower = Math.signum(yError);
            }
            else{
                yPower = 0;
            }

            POWlocation = robot.POW.getCurrentPosition();
            SOWlocation = robot.SOW.getCurrentPosition();
            BOWlocation = robot.BOW.getCurrentPosition();

            telemetry.addData("POW: ", POWlocation);
            telemetry.addData("SOW: ", SOWlocation);
            telemetry.addData("BOW: ", BOWlocation);

            telemetry.addData("robotTheta:", robotTheta);
            telemetry.addData("robotX:", robotX);
            telemetry.addData("robotY:", robotY);

            telemetry.addData("WE UPDATED", "we updated please track the changes ");
            telemetry.addData("we updated", "track the changes");
            telemetry.update();




            /*telemetry.addData("robotTheta:", robotTheta);
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

            telemetry.update();*/

            //motor use to be here
        }

        /*robot.fpd.setPower(0);
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
        telemetry.update();*/

    }

}