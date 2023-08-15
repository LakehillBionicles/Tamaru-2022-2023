package org.firstinspires.ftc.teamcode.Threemaru.Subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ExtensionSubsystem extends SubsystemBase {
    private final Servo extension;

    public enum ExtendPos {
        RETRACTED(.61), EXTENDED(0.29);

        public final double position;

        ExtendPos(double position) {
            this.position = position;
        }

        public double getPosition() {
            return position;
        }
    }

    public ExtensionSubsystem(Servo servo) {this.extension = servo;}

    public Command setExtensionPosition(double position) {return new InstantCommand(()->extension.setPosition(position));}
    public Command setExtensionExtended() {return new InstantCommand(()->extension.setPosition(ExtendPos.EXTENDED.getPosition()));}
    public Command setExtensionRetracted() {return new InstantCommand(()->extension.setPosition(ExtendPos.RETRACTED.getPosition()));}

}