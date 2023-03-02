package org.firstinspires.ftc.teamcode.Tamaru1.TeleOp1;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Tamaru1.TemaruHardware;
import org.firstinspires.ftc.teamcode.Tamaru1.Auto1.AutoBase;

@TeleOp
//@Disabled

public class fieldCentricTestOldIdea extends LinearOpMode {

    TemaruHardware robot = new TemaruHardware();


    //gamepad 1 (drive) variable
    private double drivePower;
    private double strafePower;
    private double rotatePower;



    private double fpder;
    private double bpdPower;
    private double fsdPower;
    private double bsdPower;

    private double teleDenom;

    private double fieldCentricX;
    private double fieldCentricY;

    public double POWlocation = 0;
    public double SPOWlocation = 0;
    public double SOWlocation = 0;
    public double BOWlocation = 0;

    public final double COUNTS_PER_ODO_REV = 8192;
    public final double ODO_GEAR_REDUCTION = (1.0); // This is < 1.0 if geared UP
    public final double ODO_WHEEL_DIAMETER_INCHES = 2.0;  // For figuring circumference
    public final double ODO_COUNTS_PER_INCH = ((COUNTS_PER_ODO_REV * ODO_GEAR_REDUCTION) /
            (ODO_WHEEL_DIAMETER_INCHES * 3.1415));

    public final double odoWheelGap = 11.5;

    public double robotTheta = (-(((POWlocation - SOWlocation) / ODO_COUNTS_PER_INCH) / odoWheelGap));





    public ElapsedTime runtime = new ElapsedTime();


    public void runOpMode() {

        robot.init(hardwareMap);

        waitForStart();

        telemetry.addData("update date", "____");
        telemetry.update();


        while (opModeIsActive()) {

            ///////////////////////////////////////////////////////////// GAMEPAD 1 //////////////////////////////////////////////////
            /*
            first test:
                gamepad left y =
                gamepad left x =
                gamepad right x =

            if (gamepad1.left_trigger>0) {
                drivePower = gamepad1.left_stick_y / 2;
                strafePower = -gamepad1.left_stick_x / 2;
                rotatePower = gamepad1.right_stick_x / 2;
            }else{
                drivePower = gamepad1.left_stick_y ;
                strafePower = -gamepad1.left_stick_x ;
                rotatePower = -gamepad1.right_stick_x ;
            }
*/

            ////////////////////////////////////////////////////////// MATH //////////////////////////////////////////////////////////


            fieldCentricX = (((POWlocation + SOWlocation) / 2) * Math.cos(robotTheta)) - (BOWlocation * Math.sin(robotTheta));
            fieldCentricY = (((POWlocation + SOWlocation) / 2) * Math.sin(robotTheta)) - (BOWlocation * Math.cos(robotTheta));





            ///////////////////////////////////////////////////////// MOTOR POWERS ////////////////////////////////////////////////////


            //didn't get a chance to set powers correctly
            robot.fpd.setPower(1);
            robot.bpd.setPower(1);
            robot.fsd.setPower(1);
            robot.bsd.setPower(1);







        }
    }




}