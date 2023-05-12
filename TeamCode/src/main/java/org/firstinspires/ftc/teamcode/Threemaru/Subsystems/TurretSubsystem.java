package org.firstinspires.ftc.teamcode.Threemaru.Subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TurretSubsystem extends SubsystemBase {
    private final DcMotorEx turret;

    public enum TurretPos {
        FORWARD(0), STAR(950), PORT(-950);

        public final double position;

        TurretPos(double position) {
            this.position = position;
        }

        public double getPosition() {
            return position;
        }
    }

    public TurretSubsystem (DcMotorEx motor) {this.turret = motor;}

    /*public Command setTurretForward() {return new InstantCommand(()-> setTurretPos(TurretPos.FORWARD));}
    public Command setTurretPort() {return new InstantCommand(() -> setTurretPos(TurretPos.PORT));}
    public Command setTurretStar() {return new InstantCommand(()-> setTurretPos(TurretPos.STAR));}

    public void setTurretPos(TurretPos targetPos){
        turret.setPosition(targetPos.getPosition());
    }*/

}
