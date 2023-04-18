package org.firstinspires.ftc.teamcode.Threemaru.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HandSubsystem extends SubsystemBase {
    private final Servo hand;
    public enum HandPos {
        OPEN(0), CLOSED(1);

        public final double position;

        HandPos(double position) {
            this.position = position;
        }

        public double getPosition() {
            return position;
        }
    }

    public HandSubsystem(HardwareMap HardwareMap) {
        hand = HardwareMap.get(Servo.class, "servoHand");
    }

    public void setHandPos(HandPos targetPos){
        hand.setPosition(targetPos.getPosition());
    }
}