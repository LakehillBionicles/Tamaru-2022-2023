package org.firstinspires.ftc.teamcode.Threemaru.Auto4;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Threemaru.ThreemaruVision.ConeDetection;

import java.util.HashMap;


@Config
@Autonomous(name = "TestArmCodeVision")
public class ThreemaruTestArmCodeVision extends ThreemaruAutoBase {
    @Override
    public void runOpMode() {
        super.runOpMode();
        robot.init(hardwareMap);
        HashMap<Enum, Double >ArmPositions = new HashMap<>();
        //
        ArmPositions.put(ConeDetection.RedParkingPosition.NOTSEEN, 0.0);
        ArmPositions.put(ConeDetection.RedParkingPosition.ONE, -0.6);
        ArmPositions.put(ConeDetection.RedParkingPosition.TWO, -0.5);
        ArmPositions.put(ConeDetection.RedParkingPosition.THREE, -0.4);
        ArmPositions.put(ConeDetection.RedParkingPosition.FOUR, -0.3);
        ArmPositions.put(ConeDetection.RedParkingPosition.FIVE, -0.2);
        ArmPositions.put(ConeDetection.RedParkingPosition.SIX, -0.1);
        ArmPositions.put(ConeDetection.RedParkingPosition.SEVEN, 0.1);
        ArmPositions.put(ConeDetection.RedParkingPosition.EIGHT, 0.2);
        ArmPositions.put(ConeDetection.RedParkingPosition.NINE, 0.3);
        ArmPositions.put(ConeDetection.RedParkingPosition.TEN, 0.4);
        ArmPositions.put(ConeDetection.RedParkingPosition.ELEVEN, 0.5);
        ArmPositions.put(ConeDetection.RedParkingPosition.TWELVE, 0.6);

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
                if(ArmPositions.get(ConeDetection.getRedPosition())!= null) {
                        turretToPosition(ArmPositions.get(ConeDetection.getRedPosition()));
                }
                telemetry.addData("Before colors", "yes?");
                telemetry.addData("blueColor: ", ConeDetection.getBluePosition());
                telemetry.addData("RedColor: ", ConeDetection.getRedPosition());
                telemetry.update();
            }
        }
    }
}
