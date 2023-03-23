package org.firstinspires.ftc.teamcode.Tamaru3.Auto3;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Config
@Autonomous(name = "testSensorMethods", group = "testingAuto")
public class testSensorMethods extends AutoBase {

    @Override
    public void runOpMode() {
        super.runOpMode();
        robot.init(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        correctAngle();
    }
}