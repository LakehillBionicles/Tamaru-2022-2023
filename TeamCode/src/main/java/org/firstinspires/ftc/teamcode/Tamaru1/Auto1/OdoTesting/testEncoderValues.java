package org.firstinspires.ftc.teamcode.Tamaru1.Auto1.OdoTesting;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Tamaru1.Auto1.AutoBase;

@Autonomous
//@Disabled

public class testEncoderValues extends AutoBase {

    public void runOpMode(){
        super.runOpMode();
        startUp();
        waitForStart();

        while (opModeIsActive()){


            robot.arm2.getCurrentPosition();
            telemetry.addData("encoder pos:", robot.arm2.getCurrentPosition());
            telemetry.update();







            //test color values


            //  senseColorsPort();



            //encoderDrive(0.5, 24, 24, 20);


            //sleep(1000);

            /*telemetry.addData("fpd", robot.fpd.getCurrentPosition());
            telemetry.addData("fsd", robot.fsd.getCurrentPosition());
            telemetry.addData("bpd", robot.bpd.getCurrentPosition());
            telemetry.addData("bsd", robot.bsd.getCurrentPosition());
            telemetry.addData("leftPost", robot.POW.getCurrentPosition());
            telemetry.addData("rightPost", robot.SOW.getCurrentPosition());
            telemetry.addData("frontPost", robot.BOW.getCurrentPosition());


            telemetry.update();


            sleep(10000);*/






            //stop();

        }
    }
}