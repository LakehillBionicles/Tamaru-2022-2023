package org.firstinspires.ftc.teamcode.CommandBasedTesting.Subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;
/**
 * ExtensionSubsystem
 * 1 servo
 * Command setExtensionPosition() sets the extension servo to a position
 */
public class ExtensionSubsystem extends SubsystemBase {
    private final Servo extension;

    public ExtensionSubsystem(Servo servo) {this.extension = servo;}

    public Command setExtensionPosition(double position) {return new InstantCommand(()->extension.setPosition(position));}
}
