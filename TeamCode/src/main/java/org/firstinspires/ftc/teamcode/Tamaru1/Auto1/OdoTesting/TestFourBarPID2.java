package org.firstinspires.ftc.teamcode.Tamaru1.Auto1.OdoTesting;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Tamaru1.Auto1.AutoBase;

@Autonomous
@Disabled

public class TestFourBarPID2 extends AutoBase {

    public double elbowDenominator;
    public double elbowPosition;
    public double elbowError;
    public double lastElbowError;
    public double elbowTime = 0;
    public double newElbowTarget;
    public double elbowDeriv;
    public double elbowIntegralSum = 0;
    public double elbowIntegralSumLimit = 0.25;

    public double elbowKp = 1.0;
    public double elbowKd = 1.0;
    public double elbowKi = 1.0;

    public double elbowPower;


    public void runOpMode(){
        super.runOpMode();
        startUp();
        waitForStart();

        while (opModeIsActive()){
            fourBarPID2(.5, 25, 5);
        }

    }

    public void fourBarPID2(double maxElbowPower, double inputTargetPos, double elbowTol) { //uses 2-3 predetermined targets with no way for driver to adjust
        elbowPosition = robot.arm2.getCurrentPosition();
        newElbowTarget = (inputTargetPos);
        elbowError = (newElbowTarget - elbowPosition);

        telemetry.addData("elbow pos", elbowPosition);
        telemetry.addData("new elbow target", newElbowTarget);
        telemetry.addData("error", elbowError);
        telemetry.addData("elbow time", getRuntime());
        telemetry.update();

        //robot.arm2.setPower(.5);

        while (Math.abs(elbowError) > (elbowTol)) {
            elbowError = (newElbowTarget - elbowPosition);

            elbowTime = getRuntime();

            elbowDenominator = Math.max(Math.abs(elbowPower), 1);

            //this used to be in an if loop, but it was just rechecking what got us into the while loop, so I deleted it

            elbowDeriv = ((elbowError - lastElbowError) / elbowTime); //lastElbowError=0, it is only in this file here and when it is initialized, the same is true for lastThetaError in PIDCoordinateDrive, what is the point of these?
            elbowIntegralSum = (elbowIntegralSum + (elbowError * elbowTime));

            if (elbowIntegralSum > elbowIntegralSumLimit) {
                elbowIntegralSum = elbowIntegralSumLimit;
            }
            if (elbowIntegralSum < -elbowIntegralSumLimit) {
                elbowIntegralSum = -elbowIntegralSumLimit;
            }

            elbowPower = ((elbowKd * elbowDeriv) + (elbowKi * elbowIntegralSum) + (elbowKp * Math.signum(elbowError)));
            robot.arm2.setPower(elbowPower * maxElbowPower / elbowDenominator);
            //robot.arm2.setPower(.5); //try this if still not working - didn't work

            elbowPosition = robot.arm2.getCurrentPosition();
            elbowError = (newElbowTarget - elbowPosition);
            //lastElbowError = (newElbowTarget - elbowPosition); //not sure if this is even an issue, but we never set lastElbowError to any actual value


            telemetry.addData("in while loop", "yay");
            telemetry.addData("elbowPower", elbowPower);
            telemetry.addData("elbowDenominator", elbowDenominator);
            telemetry.addData("power function", (elbowPower * maxElbowPower / elbowDenominator));
            telemetry.addData("runtime", elbowTime);
            telemetry.addData("elbow pos", elbowPosition);
            telemetry.addData("new elbow target", newElbowTarget);
            telemetry.addData("error", elbowError);
            telemetry.addData("last elbow error", lastElbowError);
            telemetry.update();
        }

        telemetry.addData("out of while loop", "yay");
        telemetry.update();

        robot.arm2.setPower(0);


    }
}