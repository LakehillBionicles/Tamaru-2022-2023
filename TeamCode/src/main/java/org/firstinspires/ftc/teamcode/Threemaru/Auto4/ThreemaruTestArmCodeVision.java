package org.firstinspires.ftc.teamcode.Threemaru.Auto4;

import static org.firstinspires.ftc.teamcode.Threemaru.Subsystems.HandSubsystem.HandPos.CLOSED1;
import static org.firstinspires.ftc.teamcode.Threemaru.Subsystems.HandSubsystem.HandPos.CLOSED2;
import static org.firstinspires.ftc.teamcode.Threemaru.Subsystems.HandSubsystem.HandPos.OPEN1;
import static org.firstinspires.ftc.teamcode.Threemaru.Subsystems.HandSubsystem.HandPos.OPEN2;
import static org.firstinspires.ftc.teamcode.Threemaru.Subsystems.TurretSubsystem.TurretPos.PORT;
import static org.firstinspires.ftc.teamcode.Threemaru.Subsystems.TurretSubsystem.TurretPos.STAR;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Threemaru.Subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.Threemaru.ThreemaruVision.ConeDetection;

import java.util.HashMap;


@Config
@Autonomous(name = "TestArmCodeVision")
public class ThreemaruTestArmCodeVision extends ThreemaruAutoBase {
    double turretPower = 0.07;
    double widthOfImage = ConeDetection.getImageWidth();
    double redConePosition;
    double distanceBetweenRedCone;

    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
        detectingCones();
        armToPosition(fiveConeArmTarget);
        robot.servoExtend.setPosition(.5);
        robot.servoHand1.setPosition(OPEN1.getPosition()); robot.servoHand2.setPosition(OPEN2.getPosition());/* open hand */
        while(!opModeIsActive()){
            trackingInitialization();
            redConePosition = ConeDetection.getRedConePosition();
            widthOfImage = ConeDetection.getImageWidth();
            while(((redConePosition-((widthOfImage/2)-51))<-10||(redConePosition-((widthOfImage/2)-51))>10)&& !opModeIsActive()){
                redConePosition = ConeDetection.getRedConePosition();
                distanceBetweenRedCone = redConePosition -((widthOfImage/2)-51);
                telemetry.addData("leftOrRight", Math.signum(distanceBetweenRedCone));
                telemetry.addData("How much left or right",distanceBetweenRedCone);
                telemetry.addData("How much left or right Without Modifier",(ConeDetection.getRedConePosition()-(widthOfImage/2)));
                telemetry.addData("blueColor: ", ConeDetection.getBlueConePosition());
                telemetry.addData("RedColor: ", ConeDetection.getRedConePosition());
                telemetry.addData("Amount of red Bars: ", ConeDetection.getRedDifferentBarAmount());
                telemetry.addData("Value of red Bars: ", ConeDetection.getRedDifferentBarvalues());

                robot.motorTurret.setPower((Math.signum(redConePosition-((widthOfImage/2)-51)))*turretPower);
                telemetry.update();
            }
            robot.motorTurret.setPower(0);
            extensionToPosition(0.01);
            sleep(3000);
            robot.servoHand1.setPosition(CLOSED1.getPosition()); robot.servoHand2.setPosition(CLOSED2.getPosition());/* open hand */
            sleep(3000);
            PIDArm(midPoleArmTarget,3);
            PIDTurret(PORT.getPosition(),2);
            sleep(1000);
            extensionToPosition(0.3);
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
    }
    public void coneTracker(){
        double widthOfImage = ConeDetection.getImageWidth();
        double redConePosition;
        double distanceBetweenRedCone;
        redConePosition = ConeDetection.getRedConePosition();
        widthOfImage = ConeDetection.getImageWidth();
        while(((redConePosition-((widthOfImage/2)-51))<-10||(redConePosition-((widthOfImage/2)-51))>10)&& !opModeIsActive()){
            redConePosition = ConeDetection.getRedConePosition();
            distanceBetweenRedCone = redConePosition -((widthOfImage/2)-51);
            telemetry.addData("leftOrRight", Math.signum(distanceBetweenRedCone));
            telemetry.addData("How much left or right",distanceBetweenRedCone);
            telemetry.addData("How much left or right Without Modifier",(ConeDetection.getRedConePosition()-(widthOfImage/2)));
            telemetry.addData("blueColor: ", ConeDetection.getBlueConePosition());
            telemetry.addData("RedColor: ", ConeDetection.getRedConePosition());
            telemetry.addData("Amount of red Bars: ", ConeDetection.getRedDifferentBarAmount());
            telemetry.addData("Value of red Bars: ", ConeDetection.getRedDifferentBarvalues());

            robot.motorTurret.setPower((Math.signum(redConePosition-((widthOfImage/2)-51)))*turretPower);
            telemetry.update();
        }
    }
    public void trackingInitialization(){
        widthOfImage = ConeDetection.getImageWidth();
        resetRuntime();
        while(!opModeIsActive()&& getRuntime()>22){
            widthOfImage = ConeDetection.getImageWidth();
            redConePosition = ConeDetection.getRedConePosition();
            distanceBetweenRedCone = redConePosition -((widthOfImage/2)-51);
            telemetry.addData("leftOrRight", Math.signum(distanceBetweenRedCone));
            telemetry.addData("How much left or right",distanceBetweenRedCone);
            telemetry.addData("How much left or right Without Modifier",(ConeDetection.getRedConePosition()-(widthOfImage/2)));
            telemetry.addData("blueColor: ", ConeDetection.getBlueConePosition());
            telemetry.addData("RedColor: ", ConeDetection.getRedConePosition());
            telemetry.addData("Amount of red Bars: ", ConeDetection.getRedDifferentBarAmount());
            telemetry.addData("Value of red Bars: ", ConeDetection.getRedDifferentBarvalues());
            telemetry.addData("Runtime", getRuntime());
            telemetry.addData("Image Width:", ConeDetection.getImageWidth());
            telemetry.update();
        }
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
