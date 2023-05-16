package org.firstinspires.ftc.teamcode.Threemaru.Auto4;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Threemaru.ThreemaruVision.ConeDetection;

import java.util.HashMap;


@Config
@Autonomous(name = "TestArmCodeVision")
public class ThreemaruTestArmCodeVision extends ThreemaruAutoBase {
    double turretPower = 0.07;

    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
        detectingCones();
        double widthOfImage = ConeDetection.getImageWidth();
        double redConePosition;
        double distanceBetweenRedCone;
        while(!opModeIsActive()){
            sleep(2000);
            redConePosition = ConeDetection.getRedConePosition();
            while((redConePosition-((widthOfImage/2)-51))<-10||(redConePosition-((widthOfImage/2)-51))>10){
                redConePosition = ConeDetection.getRedConePosition();
                distanceBetweenRedCone = redConePosition -((widthOfImage/2)-51);
                robot.motorTurret.setPower((Math.signum(redConePosition-((widthOfImage/2)-51)))*turretPower);
                telemetry.addData("leftOrRight", Math.signum(distanceBetweenRedCone));
                telemetry.addData("How much left or right",distanceBetweenRedCone);
                telemetry.addData("How much left or right Without Modifier",(ConeDetection.getRedConePosition()-(widthOfImage/2)));

            }
            robot.motorTurret.setPower(0);
            while(robot.distSensorHand.getDistance(DistanceUnit.CM)>10) {
                extensionToPosition(0.45);
            }
            telemetry.addData("leftOrRight", Math.signum(ConeDetection.getRedConePosition()-((widthOfImage/2)-51)));
            telemetry.addData("How much left or right",(ConeDetection.getRedConePosition()-((widthOfImage/2)-51)));
            telemetry.addData("How much left or right Without Modifier",(ConeDetection.getRedConePosition()-(widthOfImage/2)));

            telemetry.addData("Before colors", "yes?");
            telemetry.addData("sideOfSleeve", sideOfSleeve);
            telemetry.addData("blueColor: ", ConeDetection.getBlueConePosition());
            telemetry.addData("RedColor: ", ConeDetection.getRedConePosition());
            telemetry.addData("Amount of red Bars: ", ConeDetection.getRedDifferentBarAmount());
            telemetry.addData("Value of red Bars: ", ConeDetection.getRedDifferentBarvalues());
            /*
            if(ConeDetection.getRedDifferentBarAmount()>0) {
                telemetry.addData("Tolerance of red: ", (ConeDetection.getRedDifferentBarvalues() / ConeDetection.getRedDifferentBarAmount()));
            }

             */
            telemetry.addData("Amount of blue Bars: ", ConeDetection.getBlueDifferentBarAmount());
            telemetry.addData("Value of Blue Bars: ", ConeDetection.getBlueDifferentBarvalues());
            telemetry.addData("InputWidth: ", widthOfImage);
            /*if(ConeDetection.getBlueDifferentBarAmount()>0) {
            telemetry.addData("Tolerance of blue: ", (ConeDetection.getBlueDifferentBarvalues() / ConeDetection.getBlueDifferentBarAmount()));
            }

             */
            telemetry.update();
        }
        if (opModeIsActive()) {
            resetCamera();
            detectingCones();
            while (opModeIsActive()) {
                /*telemetry.addData("AfterWhile", "yes?");
                telemetry.update();
                //resetCamera();
                telemetry.addData("AfterresetCameraOpModeWhile", "yes?");
                telemetry.update();
                //detectingCones();
                 */
                robot.motorTurret.setPower((Math.signum(ConeDetection.getRedConePosition()-((widthOfImage/2)-51)))*turretPower);
                //telemetry.addData("leftOrRight", signum((ConeDetection.getRedDifferentPosition()-(widthOfImage-40)), 20)*turretPower);
                /*
                telemetry.addData("How much left or right",(ConeDetection.getRedDifferentPosition()-(widthOfImage-40)));
                telemetry.addData("How much left or right Without Modifier",(ConeDetection.getRedDifferentPosition()-(widthOfImage)));
                telemetry.addData("Before colors", "yes?");
                telemetry.addData("blueColor: ", ConeDetection.getBlueConePosition());
                telemetry.addData("RedColor: ", ConeDetection.getRedConePosition());
                telemetry.addData("Amount of red Bars: ", ConeDetection.getRedDifferentBarAmount());
                telemetry.addData("Value of red Bars: ", ConeDetection.getRedDifferentBarvalues());
                telemetry.addData("Amount of blue Bars: ", ConeDetection.getBlueDifferentBarAmount());
                telemetry.addData("Value of Blue Bars: ", ConeDetection.getBlueDifferentBarvalues());

                 */
                telemetry.update();
            }
        }
        camera.stopStreaming();
        camera.closeCameraDeviceAsync(() -> {});
    }
    public double  signum(double number, double tolerance){
        double a;
        if(number> tolerance){
            a= 1;
        }else if(number<-tolerance){
            a= -1;
        }else{
            a= 0;
        }
        return a;
    }

}
