package org.firstinspires.ftc.teamcode.CommandBasedTesting.Commands.ArmCommands.ArmToPole;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.CommandBasedTesting.Subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.CommandBasedTesting.Subsystems.LinearArmSubsystem;
import org.firstinspires.ftc.teamcode.CommandBasedTesting.Subsystems.TurretSubsystem;

public class armToLowPoleStar extends CommandBase {
    private final LinearArmSubsystem tamaruArm;
    private final TurretSubsystem tamaruTurret;
    private final ExtensionSubsystem tamaruExtension;

    public armToLowPoleStar(LinearArmSubsystem arm, TurretSubsystem turret, ExtensionSubsystem extension) {
        tamaruArm = arm;
        tamaruTurret = turret;
        tamaruExtension = extension;

        addRequirements(tamaruArm, tamaruTurret, tamaruExtension);
    }

    @Override
    public void execute() {
        tamaruArm.setArmToHeight(LinearArmSubsystem.Height.LOW_POLE);
        tamaruTurret.setTurretStar();
        tamaruExtension.setExtensionPosition(.4);
    }
}
