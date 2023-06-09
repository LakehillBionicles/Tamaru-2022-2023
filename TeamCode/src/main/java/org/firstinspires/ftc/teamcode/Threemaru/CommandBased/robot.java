package org.firstinspires.ftc.teamcode.Threemaru.CommandBased;

import com.arcrobotics.ftclib.command.Robot;

import org.firstinspires.ftc.teamcode.Threemaru.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Threemaru.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Threemaru.Subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.Threemaru.Subsystems.HandSubsystem;

public class robot extends Robot {

    public robot(HandSubsystem hand, ExtensionSubsystem extension, ArmSubsystem arm, DriveSubsystem drive) {
        register(hand, extension, arm, drive);
    }
}
