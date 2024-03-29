package org.firstinspires.ftc.teamcode.Threemaru.CommandBased;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.*;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp
public class CommandThreemaruTele extends BaseOpMode {

    @Override
    public void initialize(){
        super.initialize();
        CommandScheduler.getInstance().enable();
    }

    @Override
    public void run(){
        super.run();
        /////////////////////////////////ARM CONTROL///////////////////////////////////////////////////////////
        armControlButton(LEFT_BUMPER).whenPressed(hand::grab); //left bumper closes hand
        armControlButton(RIGHT_BUMPER).whenPressed(hand::release); //right bumper opens hand
        armControlButton(Y).whenPressed(extension::setExtensionExtended); //y fully extends extension
        armControlButton(B).whenPressed(extension::setExtensionRetracted); //b fully retracts extension
        armControlButton(DPAD_UP).whileHeld(new RunCommand(() -> arm.setArmPower(1)));
        armControlButton(DPAD_DOWN).whileHeld(new RunCommand(() -> arm.setArmPower(-1)));
        armControlButton(DPAD_UP).whenInactive(new RunCommand(() -> arm.setArmPower(0)));
        armControlButton(DPAD_DOWN).whenInactive(new RunCommand(() -> arm.setArmPower(0)));
        register(drive, arm, hand, extension);
        /////////////////////////////////BASE CONTROL///////////////////////////////////////////////////////////
        drive.setDefaultCommand(new RunCommand(() -> drive.setDrivePower(baseControl.getLeftY(), baseControl.getLeftX(), baseControl.getRightX())));

        CommandScheduler.getInstance().run();
    }
}