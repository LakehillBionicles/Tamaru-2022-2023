package org.firstinspires.ftc.teamcode.CommandBasedTesting.OpMode.TeleOp;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.CommandBasedTesting.Subsystems.LinearArmSubsystem;
import org.firstinspires.ftc.teamcode.CommandBasedTesting.Subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.CommandBasedTesting.Subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.CommandBasedTesting.Subsystems.HandSubsystem;
import org.firstinspires.ftc.teamcode.CommandBasedTesting.Subsystems.TurretSubsystem;
/**
 * TestBaseOpMode
 * first attempt at creating a baseOpMode using CommandOpMode
 * initializes all subsystems, gamepads, and hardware
 * used in TestTele
 */
@Disabled

public class TeleBaseOpMode extends CommandOpMode {
    public Servo servoHand, servoExtend, servoTurret;
    public DcMotorEx fpd, bpd, fsd, bsd;
    public DcMotorEx armPortI, armPortO, armStarI, armStarO;

    public HandSubsystem tamaruHand;
    public TurretSubsystem tamaruTurret;
    public ExtensionSubsystem tamaruExtension;
    public LinearArmSubsystem tamaruArm;
    public DrivetrainSubsystem tamaruDrivetrain;

    public GamepadEx baseControl;
    public GamepadEx armControl;

    @Override
    public void initialize() {
        baseControl = new GamepadEx(gamepad1);
        armControl = new GamepadEx(gamepad2);

        initHardware();

        tamaruHand = new HandSubsystem(servoHand);
        tamaruTurret = new TurretSubsystem(servoTurret);
        tamaruExtension = new ExtensionSubsystem(servoExtend);
        tamaruArm = new LinearArmSubsystem(armPortI, armPortO, armStarI, armStarO);
        tamaruDrivetrain = new DrivetrainSubsystem(fpd, bpd, fsd, bsd);

        //CommandScheduler.getInstance().enable();
    }

    @Override
    public void run() {
        super.run();
    }

    protected void initHardware() {
        servoHand = hardwareMap.get(Servo.class, "servoHand");
        servoTurret = hardwareMap.get(Servo.class, "servoTurret");
        servoExtend = hardwareMap.get(Servo.class, "servoExtend");

        fpd = hardwareMap.get(DcMotorEx.class, "fpd");
        bpd = hardwareMap.get(DcMotorEx.class, "bpd");
        fsd = hardwareMap.get(DcMotorEx.class, "fsd");
        bsd = hardwareMap.get(DcMotorEx.class, "bsd");

        armPortI = hardwareMap.get(DcMotorEx.class, "armPortI");
        armPortO = hardwareMap.get(DcMotorEx.class, "armPortO");
        armStarI = hardwareMap.get(DcMotorEx.class, "armStarI");
        armStarO = hardwareMap.get(DcMotorEx.class, "armStarO");
    }

    public GamepadButton baseControlButton(GamepadKeys.Button button){ return baseControl.getGamepadButton(button); }
    public GamepadButton armControlButton(GamepadKeys.Button button){ return armControl.getGamepadButton(button); }
}
