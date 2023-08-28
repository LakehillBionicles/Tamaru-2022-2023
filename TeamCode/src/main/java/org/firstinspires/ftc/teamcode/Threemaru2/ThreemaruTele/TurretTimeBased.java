package org.firstinspires.ftc.teamcode.Threemaru2.ThreemaruTele;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Threemaru2.Threemaru2Hardware;
@Disabled
@Autonomous
public class TurretTimeBased extends LinearOpMode {
    Threemaru2Hardware robot = new Threemaru2Hardware();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();
        if (isStopRequested()) return;
        resetRuntime();
        while(getRuntime()<1.2){
            robot.motorTurret.setPower(-.1);
        }
        robot.motorTurret.setPower(0);
    }
}
