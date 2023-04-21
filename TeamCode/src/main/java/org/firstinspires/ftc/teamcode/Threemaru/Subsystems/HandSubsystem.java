package org.firstinspires.ftc.teamcode.Threemaru.Subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HandSubsystem extends SubsystemBase {
    private final Servo hand1;
    private final Servo hand2;

    public enum HandPos {
        OPEN1(.25), CLOSED1(.6),
        OPEN2(.4), CLOSED2(0);

        public final double position;

        HandPos(double position) {
            this.position = position;
        }

        public double getPosition() {
            return position;
        }
    }

    public HandSubsystem(Servo servo1, Servo servo2) {
        this.hand1 = servo1;
        this.hand2 = servo2;
    }

    public Command grab() { return new InstantCommand(() -> setHandPos(HandPos.CLOSED1, HandPos.CLOSED2), this); }

    public Command release() { return new InstantCommand(() -> setHandPos(HandPos.OPEN1, HandPos.OPEN2), this); }


    public void setHandPos(HandPos targetPos1, HandPos targetPos2){
        hand1.setPosition(targetPos1.getPosition());
        hand2.setPosition(targetPos2.getPosition());
    }
}