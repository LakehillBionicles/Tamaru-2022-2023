package org.firstinspires.ftc.teamcode.CommandBasedTesting.OpMode.TeleOp;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.*;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandBasedTesting.Commands.ArmCommands.ArmToPole.armToMidPolePort;
import org.firstinspires.ftc.teamcode.CommandBasedTesting.Commands.ArmCommands.ArmToPole.armToMidPoleStar;
import org.firstinspires.ftc.teamcode.CommandBasedTesting.Commands.ArmCommands.armTo;
import org.firstinspires.ftc.teamcode.CommandBasedTesting.Subsystems.LinearArmSubsystem;

/** TestTele
 * attempting to use TestBaseOpMode for TeleOp
 * controls drive, arm, hand, and turret
 * baseControl is gamepad1, armControl is gamepad2
 */
@Disabled

@TeleOp
@Config
public class TestTele extends TeleBaseOpMode {


    @Override
    public void initialize() {
        super.initialize();
        CommandScheduler.getInstance().enable();
        armControlButton(DPAD_UP).whenPressed(new armTo(tamaruArm, LinearArmSubsystem.Height.LOW_POLE));
    }

    @Override
    public void run() {
        super.run();
        /*hand*/
        baseControlButton(LEFT_BUMPER).whenPressed(tamaruHand::grab);
        baseControlButton(RIGHT_BUMPER).whenPressed(tamaruHand::release);
        armControlButton(LEFT_BUMPER).whenPressed(tamaruHand::grab);
        armControlButton(RIGHT_BUMPER).whenPressed(tamaruHand::release);
        armControlButton(DPAD_UP).whenPressed(() -> schedule(new armToMidPoleStar(tamaruArm, tamaruTurret, tamaruExtension)));
        //armControlButton(DPAD_UP).whenPressed(new armTo(tamaruArm, LinearArmSubsystem.Height.LOW_POLE));
        armControlButton(DPAD_UP).whenPressed(tamaruArm.setArmToHeight(LinearArmSubsystem.Height.LOW_POLE));
        /*drive*/
        schedule(tamaruDrivetrain.drive(baseControl.getLeftY(), baseControl.getLeftX(), baseControl.getRightX()));
        /*arm*/
        tamaruArm.setDefaultCommand(tamaruArm.setArmPower(armControl.getLeftY()));
        CommandScheduler.getInstance().run();
    }
}