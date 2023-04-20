package org.firstinspires.ftc.teamcode.Threemaru.Tele4;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//@Disabled
@Config
@TeleOp
public class ServoLimits extends OpMode {
    ThreemaruHardware robot = new ThreemaruHardware();

    public static double hand1Pos = 0;
    public static double hand2Pos = 0;
    public static double extendPos = 0;
    public static double turretPos = 0;

    @Override
    public void init(){
        robot.init(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop(){
        robot.servoHand1.setPosition(hand1Pos);
        robot.servoHand2.setPosition(hand2Pos);
        robot.servoExtend.setPosition(extendPos);
        robot.servoTurret.setPosition(turretPos);

        telemetry.addData("hand1Pos", hand1Pos);
        telemetry.addData("hand2Pos", hand2Pos);
        telemetry.addData("extendPos", extendPos);
        telemetry.addData("turretPos", turretPos);
        telemetry.update();
    }
}