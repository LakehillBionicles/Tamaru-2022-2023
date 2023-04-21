package org.firstinspires.ftc.teamcode.Threemaru.CommandBased;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
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
import org.firstinspires.ftc.teamcode.Threemaru.Tele4.ThreemaruHardware;

public class BaseOpMode extends CommandOpMode {
    ThreemaruHardware robot = new ThreemaruHardware();
    /*public Servo servoHand1, servoHand2, servoExtend, servoTurret;
    public DcMotorEx fpd, bpd, fsd, bsd;
    public DcMotorEx armPort, armStar;*/

    public double drivePowerDenom = 1;

    public HandSubsystem hand;
    public TurretSubsystem turret;
    public ExtensionSubsystem extension;
    public ArmSubsystem arm;
    public DriveSubsystem drive;

    public GamepadEx baseControl;
    public GamepadEx armControl;

    @Override
    public void initialize() {
        robot.init(hardwareMap);
        baseControl = new GamepadEx(gamepad1);
        armControl = new GamepadEx(gamepad2);

        //initHardware();

        hand = new HandSubsystem(robot.servoHand1, robot.servoHand2);
        turret = new TurretSubsystem(robot.servoTurret);
        extension = new ExtensionSubsystem(robot.servoExtend);
        arm = new ArmSubsystem(robot.armPort, robot.armStar);
        drive = new DriveSubsystem(robot.fpd, robot.bpd, robot.fsd, robot.bsd);
    }

    @Override
    public void run() {
        super.run();
    }

    /*protected void initHardware() {
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
    }*/ //init hardware method

    public GamepadButton baseControlButton(GamepadKeys.Button button){ return baseControl.getGamepadButton(button); }
    public GamepadButton armControlButton(GamepadKeys.Button button){ return armControl.getGamepadButton(button); }

    public void resetTurretAndExtension() {
        extension.setExtensionRetracted();
        turret.setTurretForward();
    }

    public double getDrivePowerDenom() {
        if (gamepad1.left_trigger > 0) {
            drivePowerDenom = 2;
        } else if (gamepad1.right_trigger > 0) {
            drivePowerDenom = 4 / 3;
        } else {
            drivePowerDenom = 1;
        }
        return drivePowerDenom;
    }
}