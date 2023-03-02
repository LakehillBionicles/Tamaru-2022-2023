package org.firstinspires.ftc.teamcode.Tamaru1.Auto1.OdoTesting;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Tamaru1.Auto1.AutoBase;


//import kotlin.ranges.URangesKt;   this caused an error
@Autonomous
@Disabled
/////////////////////////////////////////////////////////DO NOT USE!!!!


public class odoWheelsFirstTest extends AutoBase {






    public void runOpMode(){
        super.runOpMode();
        startUp();
        waitForStart();

        if (opModeIsActive()){

            robot.POW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.SOW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            telemetry.addData("current pos:", robot.POW.getCurrentPosition());
            telemetry.update();

            sleep(1000);


            SPOWDriveForward(100000, .20, .01, 50000);

            sleep(50000);


            /*encoderDrive(0.5, 24, 24, 20);

            sleep(1000);

                        telemetry.addData("right ticks: ", getRightTicks());
            telemetry.addData("left ticks: ", getLeftTicks());
            telemetry.addData("front ticks: ", getFrontTicks());
            telemetry.update();


            sleep(2000000); */



            //encoderDrive(0.5, 24, 24, 20);







            //stop();

        }
    }




    public void SPOWDriveForward(double targetDistance, double targetPower, double tolerance, double timeoutS){ //if want to go backwards, change dist and power to negative

        newSPOWTarget = (((robot.POW.getCurrentPosition() + robot.SOW.getCurrentPosition()) / 2) + (int) (targetDistance));//convert target dist l8r
        SPOWlocation = ((robot.POW.getCurrentPosition() + robot.SOW.getCurrentPosition()) / 2);
        resetRuntime();


        while(runtime.seconds() < timeoutS){
            while(SPOWlocation < (1-tolerance) * newSPOWTarget){
                telemetry.addData("are we there yet?", "no");
                telemetry.addData("newSPOWTarget:", newSPOWTarget);
                telemetry.addData("SPOW location: ", SPOWlocation);
                telemetry.addData("avgEncoderPos:", ((robot.POW.getCurrentPosition() + robot.SOW.getCurrentPosition()) / 2));

                telemetry.update();

                robot.fpd.setPower(targetPower);
                robot.bpd.setPower(targetPower);
                robot.fsd.setPower(targetPower);
                robot.bsd.setPower(targetPower);

                SPOWlocation = ((robot.POW.getCurrentPosition() + robot.SOW.getCurrentPosition()) / 2); //updates the encoder's position inside the loop


            }

            //SPOWlocation = ((robot.POW.getCurrentPosition() + robot.SOW.getCurrentPosition()) / 2);

            while (SPOWlocation > (1+tolerance) * newSPOWTarget){
                telemetry.addData("are we there yet?", "too far");
                telemetry.addData("newSPOWTarget:", newSPOWTarget);
                telemetry.addData("SPOW location: ", SPOWlocation);
                telemetry.addData("avgEncoderPos:", ((robot.POW.getCurrentPosition() + robot.SOW.getCurrentPosition()) / 2));
                telemetry.update();

                robot.fpd.setPower(-(targetPower));
                robot.bpd.setPower(-(targetPower));
                robot.fsd.setPower(-(targetPower));
                robot.bsd.setPower(-(targetPower));

                SPOWlocation = ((robot.POW.getCurrentPosition() + robot.SOW.getCurrentPosition()) / 2); //updates the encoder's position inside the loop


            }

            while ((SPOWlocation > (1-tolerance) * newSPOWTarget) && (SPOWlocation < (1+tolerance) * newSPOWTarget)){
                telemetry.addData("are we there yet?", "yes");
                telemetry.addData("newSPOWTarget:", newSPOWTarget);
                telemetry.addData("SPOW location: ", SPOWlocation);
                telemetry.addData("avgEncoderPos:", ((robot.POW.getCurrentPosition() + robot.SOW.getCurrentPosition()) / 2));

                telemetry.update();

                robot.fpd.setPower(0);
                robot.bpd.setPower(0);
                robot.fsd.setPower(0);
                robot.bsd.setPower(0);

                SPOWlocation = ((robot.POW.getCurrentPosition() + robot.SOW.getCurrentPosition()) / 2); //updates the encoder's position inside the loop
            }



        }

    }


    /*public void SPOWDriveBackwards(double targetDistance, double targetPower, double tolerance, double timeoutS){

        newSPOWTarget = ((robot.POW.getCurrentPosition() + robot.SOW.getCurrentPosition()) / 2) + (int) (targetDistance);//convert target dist l8r
        SPOWlocation = ((robot.POW.getCurrentPosition() + robot.SOW.getCurrentPosition()) / 2);
        resetRuntime();


        while(runtime.seconds() < timeoutS){
            while(SPOWlocation < (1+tolerance) * newSPOWTarget){
                telemetry.addData("are we there yet?", "no");
                telemetry.addData("newSPOWTarget:", newSPOWTarget);
                telemetry.addData("SPOW location: ", SPOWlocation);
                telemetry.addData("avgEncoderPos:", ((robot.POW.getCurrentPosition() + robot.SOW.getCurrentPosition()) / 2));

                telemetry.update();

                robot.fpd.setPower(targetPower);
                robot.bpd.setPower(targetPower);
                robot.fsd.setPower(targetPower);
                robot.bsd.setPower(targetPower);

                SPOWlocation = ((robot.POW.getCurrentPosition() + robot.SOW.getCurrentPosition()) / 2); //updates the encoder's position inside the loop


            }

            SPOWlocation = ((robot.POW.getCurrentPosition() + robot.SOW.getCurrentPosition()) / 2);

            while (SPOWlocation > (1-tolerance) * newSPOWTarget){
                telemetry.addData("are we there yet?", "too far");
                telemetry.addData("newSPOWTarget:", newSPOWTarget);
                telemetry.addData("SPOW location: ", SPOWlocation);
                telemetry.addData("avgEncoderPos:", ((robot.POW.getCurrentPosition() + robot.SOW.getCurrentPosition()) / 2));
                telemetry.update();

                robot.fpd.setPower(-(targetPower));
                robot.bpd.setPower(-(targetPower));
                robot.fsd.setPower(-(targetPower));
                robot.bsd.setPower(-(targetPower));

                SPOWlocation = ((robot.POW.getCurrentPosition() + robot.SOW.getCurrentPosition()) / 2); //updates the encoder's position inside the loop


            }

            while ((SPOWlocation > (1+tolerance) * newSPOWTarget) && (SPOWlocation < (1-tolerance) * newSPOWTarget)){
                telemetry.addData("are we there yet?", "yes");
                telemetry.addData("newSPOWTarget:", newSPOWTarget);
                telemetry.addData("SPOW location: ", SPOWlocation);
                telemetry.addData("avgEncoderPos:", ((robot.POW.getCurrentPosition() + robot.SOW.getCurrentPosition()) / 2));

                telemetry.update();

                robot.fpd.setPower(0);
                robot.bpd.setPower(0);
                robot.fsd.setPower(0);
                robot.bsd.setPower(0);

                SPOWlocation = ((robot.POW.getCurrentPosition() + robot.SOW.getCurrentPosition()) / 2); //updates the encoder's position inside the loop
            }



        }

    }


    public void BOWStrafePort(double targetDistance, double targetPower, double tolerance, double timeoutS){

        newSPOWTarget = ((robot.POW.getCurrentPosition() + robot.SOW.getCurrentPosition()) / 2) + (int) (targetDistance);//convert target dist l8r
        SPOWlocation = ((robot.POW.getCurrentPosition() + robot.SOW.getCurrentPosition()) / 2);
        resetRuntime();


        while(runtime.seconds() < timeoutS){
            while(SPOWlocation < (1-tolerance) * newSPOWTarget){
                telemetry.addData("are we there yet?", "no");
                telemetry.addData("newSPOWTarget:", newSPOWTarget);
                telemetry.addData("SPOW location: ", SPOWlocation);
                telemetry.addData("avgEncoderPos:", ((robot.POW.getCurrentPosition() + robot.SOW.getCurrentPosition()) / 2));

                telemetry.update();

                robot.fpd.setPower(targetPower);
                robot.bpd.setPower(targetPower);
                robot.fsd.setPower(targetPower);
                robot.bsd.setPower(targetPower);

                SPOWlocation = ((robot.POW.getCurrentPosition() + robot.SOW.getCurrentPosition()) / 2); //updates the encoder's position inside the loop


            }

            SPOWlocation = ((robot.POW.getCurrentPosition() + robot.SOW.getCurrentPosition()) / 2);

            while (SPOWlocation > (1+tolerance) * newSPOWTarget){
                telemetry.addData("are we there yet?", "too far");
                telemetry.addData("newSPOWTarget:", newSPOWTarget);
                telemetry.addData("SPOW location: ", SPOWlocation);
                telemetry.addData("avgEncoderPos:", ((robot.POW.getCurrentPosition() + robot.SOW.getCurrentPosition()) / 2));
                telemetry.update();

                robot.fpd.setPower(-(targetPower));
                robot.bpd.setPower(-(targetPower));
                robot.fsd.setPower(-(targetPower));
                robot.bsd.setPower(-(targetPower));

                SPOWlocation = ((robot.POW.getCurrentPosition() + robot.SOW.getCurrentPosition()) / 2); //updates the encoder's position inside the loop


            }

            while ((SPOWlocation > (1-tolerance) * newSPOWTarget) && (SPOWlocation < (1+tolerance) * newSPOWTarget)){
                telemetry.addData("are we there yet?", "yes");
                telemetry.addData("newSPOWTarget:", newSPOWTarget);
                telemetry.addData("SPOW location: ", SPOWlocation);
                telemetry.addData("avgEncoderPos:", ((robot.POW.getCurrentPosition() + robot.SOW.getCurrentPosition()) / 2));

                telemetry.update();

                robot.fpd.setPower(0);
                robot.bpd.setPower(0);
                robot.fsd.setPower(0);
                robot.bsd.setPower(0);

                SPOWlocation = ((robot.POW.getCurrentPosition() + robot.SOW.getCurrentPosition()) / 2); //updates the encoder's position inside the loop
            }



        }

    }*/




}