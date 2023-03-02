package org.firstinspires.ftc.teamcode.Tamaru1.Auto1.OdoTesting;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Tamaru1.Auto1.AutoBase;

/////////////////////////////////////////////////////////DO NOT USE!!!!

//import kotlin.ranges.URangesKt;   this caused an error
@Autonomous
@Disabled

public class OdoStrafeTest extends AutoBase {






    public void runOpMode(){
        super.runOpMode();
        startUp();
        waitForStart();

        if (opModeIsActive()){

            robot.BOW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            telemetry.addData("current pos:", robot.BOW.getCurrentPosition());
            telemetry.update();

            sleep(1000);


            BOWDriveStrafeStar(-100000, -.20, .01, 50000);

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




    public void BOWDriveStrafeStar(double targetDistance, double targetPower, double tolerance, double timeoutS){  //if want to go backwards, make dist and power negative

        robot.BOW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        newBOWTarget = ((robot.BOW.getCurrentPosition()) + (int) (targetDistance));//convert target dist l8r
        BOWlocation = (robot.BOW.getCurrentPosition());
        resetRuntime();




        while(runtime.seconds() < timeoutS){
            while((Math.abs(BOWlocation)) < (1-tolerance) * (Math.abs(newBOWTarget))){
                telemetry.addData("are we there yet?", "no");
                telemetry.addData("newBOWTarget:", newBOWTarget);
                telemetry.addData("BOW location: ", BOWlocation);
                telemetry.addData("avgEncoderPos:", robot.BOW.getCurrentPosition());

                telemetry.update();

                robot.fpd.setPower(targetPower);
                robot.bpd.setPower(-(targetPower));
                robot.fsd.setPower(-(targetPower));
                robot.bsd.setPower(targetPower);

                BOWlocation = (robot.BOW.getCurrentPosition()); //updates the encoder's position inside the loop


            }

            //SPOWlocation = ((robot.POW.getCurrentPosition() + robot.SOW.getCurrentPosition()) / 2);

            while ((Math.abs(BOWlocation)) > (1+tolerance) * (Math.abs(newBOWTarget))){
                telemetry.addData("are we there yet?", "too far");
                telemetry.addData("newBOWTarget:", newBOWTarget);
                telemetry.addData("BOW location: ", BOWlocation);
                telemetry.addData("avgEncoderPos:", robot.BOW.getCurrentPosition());
                telemetry.update();

                robot.fpd.setPower(-(targetPower));
                robot.bpd.setPower(targetPower);
                robot.fsd.setPower(targetPower);
                robot.bsd.setPower(-(targetPower));

                BOWlocation = robot.BOW.getCurrentPosition(); //updates the encoder's position inside the loop


            }

            while (((Math.abs(BOWlocation)) > (1-tolerance) * (Math.abs(newBOWTarget))) && ((Math.abs(BOWlocation)) < (1+tolerance) * (Math.abs(newBOWTarget)))){
                telemetry.addData("are we there yet?", "yes");
                telemetry.addData("newBOWTarget:", newBOWTarget);
                telemetry.addData("BOW location: ", BOWlocation);
                telemetry.addData("avgEncoderPos:", robot.BOW.getCurrentPosition());
                telemetry.update();

                robot.fpd.setPower(0);
                robot.bpd.setPower(0);
                robot.fsd.setPower(0);
                robot.bsd.setPower(0);

                BOWlocation = robot.BOW.getCurrentPosition(); //updates the encoder's position inside the loop
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

