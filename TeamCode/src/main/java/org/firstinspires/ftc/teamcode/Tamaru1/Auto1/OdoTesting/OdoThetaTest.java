package org.firstinspires.ftc.teamcode.Tamaru1.Auto1.OdoTesting;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Tamaru1.Auto1.AutoBase;


//import kotlin.ranges.URangesKt;   this caused an error
/////////////////////////////////////////////////////////DO NOT USE!!!!

@Autonomous
@Disabled

public class OdoThetaTest extends AutoBase {






    public void runOpMode(){
        super.runOpMode();
        startUp();
        waitForStart();

        if (opModeIsActive()){

            robot.POW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.SOW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.BOW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);




            thetaTurn(-90, .20, 3, 50000);

            sleep(50000);






            //stop();

        }
    }




    public void thetaTurn(double degrees, double targetPower, double tolerance, double timeoutS){

        robot.POW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.SOW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BOW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        POWlocation = (robot.POW.getCurrentPosition());
        SOWlocation = (robot.SOW.getCurrentPosition());


        newTargetTheta = (Math.toRadians(degrees) + (((POWlocation - SOWlocation) / ODO_COUNTS_PER_INCH) / odoWheelGap));
        oldRobotTheta = (((POWlocation - SOWlocation) / ODO_COUNTS_PER_INCH) / odoWheelGap);

        //newSPOWTarget = (((robot.POW.getCurrentPosition() + robot.SOW.getCurrentPosition()) / 2) + (int) (targetDistance));//convert target dist l8r
        //SPOWlocation = ((robot.POW.getCurrentPosition() + robot.SOW.getCurrentPosition()) / 2);

        resetRuntime();




        while(runtime.seconds() < timeoutS){
            while((Math.abs(oldRobotTheta)) < ((Math.abs(newTargetTheta)) - Math.toRadians(tolerance))){
                POWlocation = (robot.POW.getCurrentPosition());
                SOWlocation = (robot.SOW.getCurrentPosition());
                telemetry.addData("are we there yet?", "no");
                telemetry.addData("newTargetTheta:", newTargetTheta);
                telemetry.addData("robot Theta: ", oldRobotTheta);
                telemetry.addData("robot pow: ", robot.POW.getCurrentPosition());
                telemetry.addData("robot sow: ", robot.SOW.getCurrentPosition());
                telemetry.addData("theta function:", (((POWlocation - SOWlocation) / ODO_COUNTS_PER_INCH) / odoWheelGap));

                telemetry.update();

                robot.fpd.setPower((targetPower));
                robot.bpd.setPower((targetPower));
                robot.fsd.setPower(-targetPower);
                robot.bsd.setPower(-targetPower);

                POWlocation = (robot.POW.getCurrentPosition());
                SOWlocation = (robot.SOW.getCurrentPosition());
                oldRobotTheta = (((POWlocation - SOWlocation) / ODO_COUNTS_PER_INCH) / odoWheelGap); //updates the encoder's position inside the loop


            }

            //SPOWlocation = ((robot.POW.getCurrentPosition() + robot.SOW.getCurrentPosition()) / 2);

            while ((Math.abs(oldRobotTheta)) > ((Math.abs(newTargetTheta)) + Math.toRadians(tolerance))){
                POWlocation = (robot.POW.getCurrentPosition());
                SOWlocation = (robot.SOW.getCurrentPosition());
                telemetry.addData("are we there yet?", "too far");
                telemetry.addData("newTargetTheta:", newTargetTheta);
                telemetry.addData("robot Theta: ", oldRobotTheta);
                telemetry.addData("theta function:", (((POWlocation - SOWlocation) / ODO_COUNTS_PER_INCH) / odoWheelGap));
                telemetry.update();

                robot.fpd.setPower(-targetPower);
                robot.bpd.setPower(-targetPower);
                robot.fsd.setPower((targetPower));
                robot.bsd.setPower((targetPower));

                POWlocation = (robot.POW.getCurrentPosition());
                SOWlocation = (robot.SOW.getCurrentPosition());
                oldRobotTheta = (((POWlocation - SOWlocation) / ODO_COUNTS_PER_INCH) / odoWheelGap); //updates the encoder's position inside the loop


            }

            while (((Math.abs(oldRobotTheta)) > ((Math.abs(newTargetTheta)) - Math.toRadians(tolerance))) && ((Math.abs(oldRobotTheta)) < ((Math.abs(newTargetTheta)) + Math.toRadians(tolerance)))){
                POWlocation = (robot.POW.getCurrentPosition());
                SOWlocation = (robot.SOW.getCurrentPosition());
                telemetry.addData("are we there yet?", "yes");
                telemetry.addData("newTargetTheta:", newTargetTheta);
                telemetry.addData("robot Theta: ", oldRobotTheta);
                telemetry.addData("theta function:", (((POWlocation - SOWlocation) / ODO_COUNTS_PER_INCH) / odoWheelGap));
                telemetry.update();

                robot.fpd.setPower(0);
                robot.bpd.setPower(0);
                robot.fsd.setPower(0);
                robot.bsd.setPower(0);

                POWlocation = (robot.POW.getCurrentPosition());
                SOWlocation = (robot.SOW.getCurrentPosition());
                oldRobotTheta = (((POWlocation - SOWlocation) / ODO_COUNTS_PER_INCH) / odoWheelGap); //updates the encoder's position inside the loop
            }



        }

    }


}
