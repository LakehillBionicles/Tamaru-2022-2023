package org.firstinspires.ftc.teamcode.Threemaru.Auto4;

import static org.firstinspires.ftc.teamcode.Threemaru.Subsystems.ArmSubsystem.Height.HIGH_POLE;
import static org.firstinspires.ftc.teamcode.Threemaru.Subsystems.HandSubsystem.HandPos.*;
import static org.firstinspires.ftc.teamcode.Threemaru.Subsystems.TurretSubsystem.TurretPos.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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

        if (opModeIsActive()) {
            ControlAll(75, 0, HIGH_POLE.getHeight(), FORWARD.getPosition(), 3);/* drive past high pole */
            distDriveStar(-1, 3);/* back to high pole */
            PIDTurret(STAR.getPosition(), 2);/* turret star */
            double dist = robot.distSensorStar.getDistance(DistanceUnit.CM);
            double extendPosition = Math.max(1.92 + -0.126*dist + 2.23E-03*dist*dist, 0);
            robot.servoExtend.setPosition(extendPosition);
            telemetry.addData("extendPos", extendPosition);
            telemetry.update();
            sleep(1000);/* wait for extension */
            robot.servoHand1.setPosition(OPEN1.getPosition()); robot.servoHand2.setPosition(OPEN2.getPosition());/* open hand */
            sleep(1000);/* wait for cone to fall */
            PIDTurret(FORWARD.getPosition(),2);/* turret forward */
        }
    }
}