package org.firstinspires.ftc.teamcode.Threemaru.Tele4;

import static org.firstinspires.ftc.teamcode.Threemaru.Subsystems.TurretSubsystem.TurretPos.*;
import static org.firstinspires.ftc.teamcode.Threemaru.Subsystems.HandSubsystem.HandPos.*;
import static org.firstinspires.ftc.teamcode.Threemaru.Subsystems.ExtensionSubsystem.ExtendPos.*;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Threemaru.Subsystems.HandSubsystem;
import org.firstinspires.ftc.teamcode.Threemaru.Subsystems.TurretSubsystem;

@TeleOp
//@Disabled
public class ThreemaruTele extends LinearOpMode {
    ThreemaruHardware robot = new ThreemaruHardware();
    public double drivePower, strafePower, rotPower, armPower;
    public double extendPosition = RETRACTED.getPosition();
    public int drivePowerDenom = 1;
    public HandSubsystem.HandPos handPos1 = OPEN1;
    public HandSubsystem.HandPos handPos2 = OPEN2;
    public TurretSubsystem.TurretPos turretPosition = FORWARD;

    public void runOpMode() {
        robot.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            robot.fpd.setPower(-gamepad2.left_stick_y - gamepad2.left_stick_x + gamepad2.right_stick_x);
            robot.bpd.setPower(-gamepad2.left_stick_y + gamepad2.left_stick_x + gamepad2.right_stick_x);
            robot.fsd.setPower(-gamepad2.left_stick_y + gamepad2.left_stick_x - gamepad2.right_stick_x);
            robot.bsd.setPower(-gamepad2.left_stick_y - gamepad2.left_stick_x - gamepad2.right_stick_x);

            robot.armStar.setPower(getArmPower());
            robot.armPort.setPower(getArmPower());

            robot.servoHand1.setPosition(getHandPos1().getPosition());
            robot.servoHand2.setPosition(getHandPos2().getPosition());
            robot.servoExtend.setPosition(getExtendPosition());
            //robot.servoTurret.setPosition(getTurretPosition().getPosition());
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
        if ((gamepad1.dpad_up && (robot.distSensorPort.getDistance(DistanceUnit.CM) > 10))) {
            drivePower = .175;
        } else if ((gamepad1.dpad_down && (robot.distSensorPort.getDistance(DistanceUnit.CM) > 10))) {
            drivePower = -.175;
        } else if ((gamepad1.dpad_right && (robot.distSensorStar.getDistance(DistanceUnit.CM) > 10))) {
            drivePower = .175;
        } else if ((gamepad1.dpad_left && (robot.distSensorStar.getDistance(DistanceUnit.CM) > 10))) {
            drivePower = -.175;
        } else {
            drivePower = -gamepad2.left_stick_y/getDrivePowerDenom();
        }
        return drivePower;
    }

    public HandSubsystem.HandPos getHandPos1() {
        if (gamepad1.left_bumper) {
            handPos1 = CLOSED1;
        }

        if (gamepad2.left_bumper) {
            handPos1 = CLOSED1;
        } else if (gamepad2.right_bumper) {
            handPos1 = OPEN1;
        }

        return handPos1;
    }

    public HandSubsystem.HandPos getHandPos2() {
        if (gamepad1.left_bumper) {
            handPos2 = CLOSED2;
        }

        if (gamepad2.left_bumper) {
            handPos2 = CLOSED2;
        } else if (gamepad2.right_bumper) {
            handPos2 = OPEN2;
        }

        return handPos2;
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
        /*if (turretPosition == PORT) {
            double distPort = (robot.distSensorPort.getDistance(DistanceUnit.CM));
            extendPosition = Math.max(1.33 + -0.188 * distPort + 0.0104 * distPort * distPort, .5);
        } else if (turretPosition == STAR) {
            double distStar = robot.distSensorStar.getDistance(DistanceUnit.CM);
            extendPosition = Math.max(1.6 + -0.145 * distStar + 4.76E-04 * distStar * distStar, .5);
        }*/

        if (gamepad2.dpad_down) {
            extendPosition = RETRACTED.getPosition();
        }

        if (gamepad2.y) {
            extendPosition = EXTENDED.getPosition();
        } else if (gamepad2.b) {
            extendPosition = RETRACTED.getPosition();
        }

        return extendPosition;
    }

    public double getArmPower(){
        if(gamepad2.dpad_up){
            armPower = 1;
        } else if(gamepad2.dpad_down){
            armPower = -1;
        } else {
            armPower = 0;
        }
        return armPower;
    }
}