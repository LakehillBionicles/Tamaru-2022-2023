package org.firstinspires.ftc.teamcode.Threemaru;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandBasedTesting.Subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.CommandBasedTesting.Subsystems.LinearArmSubsystem;
import org.firstinspires.ftc.teamcode.Threemaru.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Threemaru.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Threemaru.Subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.Threemaru.Subsystems.HandSubsystem;
import org.firstinspires.ftc.teamcode.Threemaru.Subsystems.TurretSubsystem;

public class ThreemaruBase extends CommandOpMode {
    public DistanceSensor distSensorHand, distSensorStar, distSensorPort;

    public DriveSubsystem ThreemaruDrive;
    public ArmSubsystem ThreemaruArm;
    public HandSubsystem ThreemaruHand;
    public ExtensionSubsystem ThreemaruExtension;
    public TurretSubsystem ThreemaruTurret;

    public GamepadEx baseControl;
    public GamepadEx armControl;
    private final ElapsedTime runtime = new ElapsedTime();

    HardwareMap hwMap = null;

    @Override
    public void initialize() {
        baseControl = new GamepadEx(gamepad1);
        armControl = new GamepadEx(gamepad2);

        ThreemaruDrive = new DriveSubsystem(hwMap);
        ThreemaruArm = new ArmSubsystem(hwMap);
        ThreemaruHand = new HandSubsystem(hwMap);
        ThreemaruExtension = new ExtensionSubsystem(hwMap);
        ThreemaruTurret = new TurretSubsystem(hwMap);

        distSensorHand = hwMap.get(DistanceSensor.class, "distSensorHand");
        distSensorPort = hwMap.get(DistanceSensor.class, "distSensorPort");
        distSensorStar = hwMap.get(DistanceSensor.class, "distSensorStar");
    }

    @Override
    public void run() {
        super.run();
    }

    public GamepadButton baseControlButton(GamepadKeys.Button button){ return baseControl.getGamepadButton(button); }
    public GamepadButton armControlButton(GamepadKeys.Button button){ return armControl.getGamepadButton(button); }
}