package org.firstinspires.ftc.teamcode.Threemaru.Auto4;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Threemaru.ThreemaruVision.ConeDetection;

import java.util.HashMap;


@Config
@Autonomous(name = "TestArmCodeVision")
public class ThreemaruTestArmCodeVision extends ThreemaruAutoBase {
    double turretPower = 0.3;
    @Override
    public void runOpMode() {
        super.runOpMode();
        robot.init(hardwareMap);
        resetArm();
        resetDrive();
        resetCamera();
        detectingCones();
        while(!opModeIsActive()){
            telemetry.addData("sideOfSleeve", sideOfSleeve);
            telemetry.addData("blueColor: ", ConeDetection.getBluePosition());
            telemetry.addData("RedColor: ", ConeDetection.getRedPosition());
            telemetry.update();
        }
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                /*telemetry.addData("AfterWhile", "yes?");
                telemetry.update();
                //resetCamera();
                telemetry.addData("AfterresetCameraOpModeWhile", "yes?");
                telemetry.update();
                //detectingCones();
                 */
                robot.motorTurret.setVelocity(Math.signum(ConeDetection.getRedDifferentPosition()-ConeDetection.getImageWidth())/turretPower);
                telemetry.addData("Before colors", "yes?");
                telemetry.addData("blueColor: ", ConeDetection.getBluePosition());
                telemetry.addData("RedColor: ", ConeDetection.getRedPosition());
                telemetry.update();
            }
        }
    }
}
