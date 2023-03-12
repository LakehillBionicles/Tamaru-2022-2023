package org.firstinspires.ftc.teamcode.CommandBasedTesting.Commands.ArmCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.CommandBasedTesting.Subsystems.LinearArmSubsystem;

public class armTo extends CommandBase {

    private final LinearArmSubsystem m_arm;
    private final LinearArmSubsystem.Height m_height;

    public armTo(LinearArmSubsystem arm, LinearArmSubsystem.Height height){
        m_arm = arm;
        m_height = height;
        addRequirements(m_arm);
    }

    @Override
    public void execute() {
        m_arm.setArmTarget(m_height.getHeight());
        m_arm.setArmMode(DcMotor.RunMode.RUN_TO_POSITION);
        m_arm.setArmPower(1);
    }
}