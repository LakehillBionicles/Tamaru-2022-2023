package org.firstinspires.ftc.teamcode.Threemaru.Tele4;

import static org.firstinspires.ftc.teamcode.Threemaru.Subsystems.TurretSubsystem.TurretPos.*;
import static org.firstinspires.ftc.teamcode.Threemaru.Subsystems.HandSubsystem.HandPos.*;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Threemaru.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Threemaru.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Threemaru.Subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.Threemaru.Subsystems.HandSubsystem;
import org.firstinspires.ftc.teamcode.Threemaru.Subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.Threemaru.ThreemaruBase;

@TeleOp
//@Disabled
public class ThreemaruTele extends ThreemaruBase {
    public double drivePower, strafePower, rotPower;
    public double extendPosition;
    public int drivePowerDenom;
    public HandSubsystem.HandPos handPos;
    public TurretSubsystem.TurretPos turretPosition;

    @Override
    public void initialize() { super.initialize(); }

    public void runOpMode() {
        waitForStart();

        while (opModeIsActive()) {
            ThreemaruDrive.setDrivePower(getDrivePower(), -gamepad1.left_stick_y/getDrivePowerDenom(), rotPower = gamepad1.right_stick_x/getDrivePowerDenom());
            ThreemaruArm.setArmPower(-gamepad1.left_stick_y);
            ThreemaruHand.setHandPos(getHandPos());
            ThreemaruExtension.setExtensionPos(getExtendPosition());
            ThreemaruTurret.setTurretPos(getTurretPosition());
        }
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

    public double getDrivePower() {
        if ((gamepad1.dpad_up && (distSensorPort.getDistance(DistanceUnit.CM) > 10))) {
            drivePower = .175;
        } else if ((gamepad1.dpad_down && (distSensorPort.getDistance(DistanceUnit.CM) > 10))) {
            drivePower = -.175;
        } else if ((gamepad1.dpad_right && (distSensorStar.getDistance(DistanceUnit.CM) > 10))) {
            drivePower = .175;
        } else if ((gamepad1.dpad_left && (distSensorStar.getDistance(DistanceUnit.CM) > 10))) {
            drivePower = -.175;
        } else {
            drivePower = -gamepad1.left_stick_y/getDrivePowerDenom();
        }
        return drivePower;
    }

    public HandSubsystem.HandPos getHandPos() {
        if (gamepad1.left_bumper) {
            handPos = CLOSED;
        }

        if (gamepad2.left_bumper) {
            handPos = CLOSED;
        } else if (gamepad2.right_bumper) {
            handPos = OPEN;
        }

        return handPos;
    }

    public TurretSubsystem.TurretPos getTurretPosition() {
        if (gamepad2.dpad_up) {
            turretPosition = FORWARD;
        } else if (gamepad2.dpad_left) {
            turretPosition = STAR;
        } else if (gamepad2.dpad_right) {
            turretPosition = PORT;
        } else if (gamepad2.dpad_down) {
            turretPosition = FORWARD;
        }
        return turretPosition;
    }

    public double getExtendPosition() {
        if (turretPosition == PORT) {
            double distPort = (distSensorPort.getDistance(DistanceUnit.CM));
            extendPosition = Math.max(1.33 + -0.188 * distPort + 0.0104 * distPort * distPort, .5);
        } else if (turretPosition == STAR) {
            double distStar = distSensorStar.getDistance(DistanceUnit.CM);
            extendPosition = Math.max(1.6 + -0.145 * distStar + 4.76E-04 * distStar * distStar, .5);
        }

        if (gamepad2.dpad_down) {
            extendPosition = 1;
        }

        if (gamepad2.y) {
            extendPosition = 1;
        } else if (gamepad2.b) {
            extendPosition = 0.75;
        } else if (gamepad2.a) {
            extendPosition = 0.5;
        } else if (gamepad2.x) {
            extendPosition = .25;
        }

        return extendPosition;
    }
}