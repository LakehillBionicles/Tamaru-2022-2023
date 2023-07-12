package org.firstinspires.ftc.teamcode.Threemaru.Tamaru3.Auto3.OldPaths;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Threemaru.Tamaru3.Auto3.AutoBase;

@Config
@Disabled
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