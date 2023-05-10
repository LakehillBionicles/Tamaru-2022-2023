package org.firstinspires.ftc.teamcode.Threemaru.Auto4;

import static org.firstinspires.ftc.teamcode.Threemaru.Subsystems.ArmSubsystem.Height.HIGH_POLE;
import static org.firstinspires.ftc.teamcode.Threemaru.Subsystems.HandSubsystem.HandPos.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//@Disabled
@Config
@Autonomous(name = "TestAuto")
public class ThreemaruTestAuto extends ThreemaruAutoBase {
    @Override
    public void runOpMode() {
        super.runOpMode();
        robot.init(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        initAuto();

        waitForStart();

        while (opModeIsActive()) {
            ControlAll(75, 0, HIGH_POLE.getHeight(), turretFront, 3);
            distDriveStar(-1, 3);
            ControlAll(0, 0, HIGH_POLE.getHeight(), turretStar, 2);
            extensionToPositionStar();
            //robot.servoHand1.setPosition(OPEN1.getPosition()); robot.servoHand2.setPosition(OPEN2.getPosition());
            //PIDTurret(turretFront,5);
            //PIDDrive(-12,0, 5);
            /*PIDDrive(0, 90, 2);
            PIDDrive(10, 5);
            distDrivePort(1, 10);
            if (sideOfSleeve == 1) {
                PIDDrive(12, 5);
            } else if (sideOfSleeve == 2) {
                PIDDrive(-6, 5);
            } else {
                PIDDrive(-30, 5);
            }*/
        }
    }
}