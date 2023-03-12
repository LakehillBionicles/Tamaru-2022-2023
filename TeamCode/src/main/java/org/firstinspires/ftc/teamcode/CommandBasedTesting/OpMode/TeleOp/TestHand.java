package org.firstinspires.ftc.teamcode.CommandBasedTesting.OpMode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.*;

import org.firstinspires.ftc.teamcode.CommandBasedTesting.Subsystems.HandSubsystem;
/**
 * TestHand (TeleOp)
 * first successful attempt at using CommandOpMode
 * opens and closes the hand
 */
@TeleOp
@Config
public class TestHand extends CommandOpMode {
    protected Servo servoHand;
    protected HandSubsystem tamaruHand;
    protected GamepadEx gamepadEx2;

    @Override
    public void initialize() {
        //super.initialize();
        gamepadEx2 = new GamepadEx(gamepad2);

        initHardware();

        tamaruHand = new HandSubsystem(servoHand);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void run() {
        super.run();
        //armControl(LEFT_BUMPER).whenPressed(tamaruHand.grab());
        armControl(LEFT_BUMPER).whenPressed(new InstantCommand(() -> telemetry.update()));
        armControl(RIGHT_BUMPER).whenPressed(tamaruHand.release());
    }

    protected void initHardware() {
        servoHand = hardwareMap.get(Servo.class, "servoHand");
    }

    protected GamepadButton armControl(GamepadKeys.Button button){ return gamepadEx2.getGamepadButton(button); }

}