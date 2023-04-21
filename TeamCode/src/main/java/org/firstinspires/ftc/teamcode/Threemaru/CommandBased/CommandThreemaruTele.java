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
        drive.setDrivePower(-armControl.getLeftY()/getDrivePowerDenom(),
                armControl.getLeftX()/getDrivePowerDenom(),
                armControl.getRightX()/getDrivePowerDenom()); //drive power set by sticks
        //armControlButton(DPAD_UP).whenPressed(turret::setTurretForward); //DPAD_UP sets turret forward
        //armControlButton(DPAD_LEFT).whenPressed(turret::setTurretPort); //DPAD_LEFT sets turret port
        //armControlButton(DPAD_RIGHT).whenPressed(turret::setTurretStar); //DPAD_RIGHT sets turret star
        //armControlButton(DPAD_DOWN).whenPressed(new InstantCommand(this::resetTurretAndExtension));
        //arm.setArmPower(-armControl.getLeftY()); //arm power is set to leftStickY
        /////////////////////////////////BASE CONTROL///////////////////////////////////////////////////////////
        /*baseControlButton(LEFT_BUMPER).whenPressed(hand::grab); //left bumper closed hand
        drive.setDrivePower(-baseControl.getLeftY()/getDrivePowerDenom(),
                baseControl.getLeftX()/getDrivePowerDenom(),
                baseControl.getRightX()/getDrivePowerDenom()); //drive power set by sticks*/
    }
}