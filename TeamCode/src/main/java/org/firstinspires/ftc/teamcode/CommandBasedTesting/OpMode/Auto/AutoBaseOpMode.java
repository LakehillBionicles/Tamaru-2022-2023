package org.firstinspires.ftc.teamcode.CommandBasedTesting.OpMode.Auto;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
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
public class AutoBaseOpMode extends CommandOpMode {
    protected Servo servoHand, servoExtend, servoTurret;
    protected DcMotorEx fpd, bpd, fsd, bsd;
    protected DcMotorEx armPortI, armPortO, armStarI, armStarO;

    protected HandSubsystem tamaruHand;
    protected TurretSubsystem tamaruTurret;
    protected ExtensionSubsystem tamaruExtension;
    protected LinearArmSubsystem tamaruArm;
    protected DrivetrainSubsystem tamaruDrivetrain;

    @Override
    public void initialize() {
        initHardware();

        tamaruHand = new HandSubsystem(servoHand);
        tamaruTurret = new TurretSubsystem(servoTurret);
        tamaruExtension = new ExtensionSubsystem(servoExtend);
        tamaruArm = new LinearArmSubsystem(armPortI, armPortO, armStarI, armStarO);
        tamaruDrivetrain = new DrivetrainSubsystem(fpd, bpd, fsd, bsd);
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
}
