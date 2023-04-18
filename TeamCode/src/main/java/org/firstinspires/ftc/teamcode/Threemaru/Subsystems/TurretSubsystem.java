package org.firstinspires.ftc.teamcode.Threemaru.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TurretSubsystem extends SubsystemBase {
    private final Servo turret;

    public enum TurretPos {
        FORWARD(.5), STAR(1), PORT(0);

        public final double position;

        TurretPos(double position) {
            this.position = position;
        }

        public double getPosition() {
            return position;
        }
    }

    public TurretSubsystem (HardwareMap HardwareMap) {
        turret = HardwareMap.get(Servo.class, "servoTurret");
    }

    public void setTurretPos(TurretPos targetPos){
        turret.setPosition(targetPos.getPosition());
    }

}
