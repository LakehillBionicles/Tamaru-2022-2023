package org.firstinspires.ftc.teamcode.Threemaru.Auto4;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Threemaru.ThreemaruVision.ConeDetection;

import java.util.HashMap;


@Config
@TeleOp(name = "TestArmCodeVision")
public class ThreemaruTestArmCodeVision extends ThreemaruAutoBase {
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
            telemetry.addData("blueColor: ", ConeDetection.getBlueDifferentPosition());
            telemetry.addData("RedColor: ", ConeDetection.getRedDifferentPosition());
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
                //Add equation to find correct position
                armToPosition((ConeDetection.getBlueDifferentPosition()));
                extensionToPosition(ConeDetection.getBlueDistance());
                telemetry.addData("Before colors", "yes?");
                telemetry.addData("blueColor: ", ConeDetection.getBlueDifferentPosition());
                telemetry.addData("RedColor: ", ConeDetection.getRedDifferentPosition());
                telemetry.update();
            }
        }
    }
}
