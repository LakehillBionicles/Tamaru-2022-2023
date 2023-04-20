package org.firstinspires.ftc.teamcode.Threemaru.CommandBased;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Threemaru.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Threemaru.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Threemaru.Subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.Threemaru.Subsystems.HandSubsystem;
import org.firstinspires.ftc.teamcode.Threemaru.Subsystems.TurretSubsystem;

public class BaseOpMode extends CommandOpMode {
    public Servo servoHand1, servoHand2, servoExtend, servoTurret;
    public DcMotorEx fpd, bpd, fsd, bsd;
    public DcMotorEx armPort, armStar;

    public HandSubsystem ThreemaruHand;
    public TurretSubsystem ThreemaruTurret;
    public ExtensionSubsystem ThreemaruExtension;
    public ArmSubsystem ThreemaruArm;
    public DriveSubsystem ThreemaruDrive;

    public GamepadEx baseControl;
    public GamepadEx armControl;

    @Override
    public void initialize() {
        baseControl = new GamepadEx(gamepad1);
        armControl = new GamepadEx(gamepad2);

        initHardware();

        ThreemaruHand = new HandSubsystem(servoHand1, servoHand2);
        ThreemaruTurret = new TurretSubsystem(servoTurret);
        ThreemaruExtension = new ExtensionSubsystem(servoExtend);
        ThreemaruArm = new ArmSubsystem(armPort, armStar);
        ThreemaruDrive = new DriveSubsystem(fpd, bpd, fsd, bsd);
    }

    @Override
    public void run() {
        super.run();
    }

    protected void initHardware() {
        servoHand1 = hardwareMap.get(Servo.class, "servoHand1");
        servoHand2 = hardwareMap.get(Servo.class, "servoHand2");
        servoTurret = hardwareMap.get(Servo.class, "servoTurret");
        servoExtend = hardwareMap.get(Servo.class, "servoExtend");

        fpd = hardwareMap.get(DcMotorEx.class, "fpd");
        bpd = hardwareMap.get(DcMotorEx.class, "bpd");
        fsd = hardwareMap.get(DcMotorEx.class, "fsd");
        bsd = hardwareMap.get(DcMotorEx.class, "bsd");

        armPort = hardwareMap.get(DcMotorEx.class, "armPort");
        armStar = hardwareMap.get(DcMotorEx.class, "armStar");
    }

    public GamepadButton baseControlButton(GamepadKeys.Button button){ return baseControl.getGamepadButton(button); }
    public GamepadButton armControlButton(GamepadKeys.Button button){ return armControl.getGamepadButton(button); }
}
