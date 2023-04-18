package org.firstinspires.ftc.teamcode.Threemaru.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ExtensionSubsystem extends SubsystemBase {
    private final Servo extension;

    public ExtensionSubsystem(HardwareMap HardwareMap) {
        extension = HardwareMap.get(Servo.class, "servoExtend");
    }

    public void setExtensionPos(double targetPos){
        extension.setPosition(targetPos);
    }
}
