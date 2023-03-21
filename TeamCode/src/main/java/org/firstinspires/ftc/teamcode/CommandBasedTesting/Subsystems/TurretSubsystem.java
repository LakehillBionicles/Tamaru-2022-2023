package org.firstinspires.ftc.teamcode.CommandBasedTesting.Subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * TurretSubsystem
 * 1 servo
 * Command setTurretForward() returns instant command to set turret position to forward
 * Command setTurretPort() returns instant command to set turret position to port
 * Command setTurretStar() returns instant command to set turret position to star
 * */
public class TurretSubsystem extends SubsystemBase {
    private final Servo turret;
    public static double turretForward = .275;
    public static double turretPort = 0;
    public static double turretStar = .7;

    public TurretSubsystem (Servo servo) {this.turret = servo;}

    public Command setTurretForward() {return new InstantCommand(()->turret.setPosition(turretForward));}
    public Command setTurretPort() {return new InstantCommand(() -> turret.setPosition(turretPort));}
    public Command setTurretStar() {return new InstantCommand(()->turret.setPosition(turretStar));}

}
