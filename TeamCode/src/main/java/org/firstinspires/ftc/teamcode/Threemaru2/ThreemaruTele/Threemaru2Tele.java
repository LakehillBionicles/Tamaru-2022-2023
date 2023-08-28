package org.firstinspires.ftc.teamcode.Threemaru2.ThreemaruTele;

import static org.firstinspires.ftc.teamcode.Threemaru.Subsystems.TurretSubsystem.TurretPos.*;
import static org.firstinspires.ftc.teamcode.Threemaru.Subsystems.HandSubsystem.HandPos.*;
import static org.firstinspires.ftc.teamcode.Threemaru.Subsystems.ExtensionSubsystem.ExtendPos.*;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Threemaru.Subsystems.HandSubsystem;
import org.firstinspires.ftc.teamcode.Threemaru.Subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.Threemaru2.Threemaru2Hardware;

@TeleOp
//@Disabled
public class Threemaru2Tele extends LinearOpMode {
    Threemaru2Hardware robot = new Threemaru2Hardware();
    public double drivePower, strafePower, rotPower, armPower;
    public double extendPosition = RETRACTED.getPosition();
    public int drivePowerDenom = 1;
    public int turretTarget = 0, turretPos;
    public HandSubsystem.HandPos handPos1 = OPEN1;
    public HandSubsystem.HandPos handPos2 = OPEN2;
    public TurretSubsystem.TurretPos turretPosition = FORWARD;
    private PIDController armController, turretController;
    public static double p = 0.001, i = 0, d = 0.0001, kg = 0.001;
    public static double pTurret = 0.0005, iTurret = 0.00001, dTurret = 0.0001;
    public static double reference = 0;
    public static double maxVelocity = 4000;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();
    public void runOpMode() {
        robot.init(hardwareMap);
        armController = new PIDController(p, i, d);
        turretController = new PIDController(pTurret, iTurret, dTurret);

        robot.armPort.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armStar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorTurret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.armPort.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.armStar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorTurret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turretController.setPID(pTurret, iTurret, dTurret);

        waitForStart();

        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            robot.fpd.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x)/getDrivePowerDenom());
            robot.bpd.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x)/getDrivePowerDenom());
            robot.fsd.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x)/getDrivePowerDenom());
            robot.bsd.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x)/getDrivePowerDenom());

            robot.armStar.setPower(getArmPower());
            robot.armPort.setPower(getArmPower());

            turretController.setSetPoint(getTurretTarget());
            double turretPos = robot.motorTurret.getCurrentPosition();
            robot.motorTurret.setPower(turretController.calculate(turretPos, turretController.getSetPoint()));

            robot.servoHand1.setPosition(getHandPos1().getPosition());
            robot.servoHand2.setPosition(getHandPos2().getPosition());
            robot.servoExtend.setPosition(getExtendPosition());

            telemetry.addData("turret pos", robot.motorTurret.getCurrentPosition());
            telemetry.update();
        }
    }
    public void distDriveStar(int direction, double timeout){
        resetRuntime();
        while(robot.distSensorStarB.getDistance(DistanceUnit.CM)>50 && getRuntime()<timeout){
            robot.fpd.setPower(direction*.3);
            robot.bpd.setPower(direction*.3);
            robot.fsd.setPower(direction*.3);
            robot.bsd.setPower(direction*.3);

            telemetry.addData("distStar1", robot.distSensorStarB.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
        robot.fpd.setPower(0);
        robot.bpd.setPower(0);
        robot.fsd.setPower(0);
        robot.bsd.setPower(0);
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
        if ((gamepad2.dpad_up && (robot.distSensorPort.getDistance(DistanceUnit.CM) > 10))) {
            drivePower = .175;
        } else if ((gamepad1.dpad_down && (robot.distSensorPort.getDistance(DistanceUnit.CM) > 10))) {
            drivePower = -.175;
        } else if ((gamepad1.dpad_right && (robot.distSensorStarB.getDistance(DistanceUnit.CM) > 10))) {
            drivePower = .175;
        } else if ((gamepad1.dpad_left && (robot.distSensorStarB.getDistance(DistanceUnit.CM) > 10))) {
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
    public double getTurretTarget(){
        if(gamepad2.dpad_right){
            turretTarget = 2500;
        } else if(gamepad2.dpad_left){
            turretTarget = -2500;
        } else if(gamepad2.dpad_up){
            turretTarget = 0;
        }

        return turretTarget;
    }
    public void PIDTurret(double target) {
        turretController.setPID(pTurret, iTurret, dTurret);

        turretController.setSetPoint(target);

        turretController.setTolerance(1);

        double turretPos = robot.motorTurret.getCurrentPosition();

        resetRuntime();
        while (!turretController.atSetPoint()) {
            turretPos = robot.motorTurret.getCurrentPosition();

            double pid = turretController.calculate(turretPos, turretController.getSetPoint());
            robot.motorTurret.setPower(pid);
        }
        robot.motorTurret.setPower(0);
    }
    public TurretSubsystem.TurretPos getTurretPosition() {
        if (gamepad2.dpad_up) {
            turretPosition = FORWARD;
        } else if (gamepad2.dpad_right) {
            turretPosition = STAR;
        } else if (gamepad2.dpad_left) {
            turretPosition = PORT;
        } else if (gamepad2.dpad_down) {
            turretPosition = FORWARD;
        }
        return turretPosition;
    }
    public double getExtendPosition() {

        double distPort = (robot.distSensorPort.getDistance(DistanceUnit.CM));
        double distStar = robot.distSensorStarB.getDistance(DistanceUnit.CM);
        //extendPosition = Math.max((0.735 + -0.0247 * distPort + 3.31E-04 *distPort *distPort), .29);
        //extendPosition = Math.max((0.67 + -0.0152 * distStar + 9.16E-05 * distStar * distStar)-.05, .29);
        //extendPosition = Math.max((0.668 + -0.0195 * distStar + 2.54E-04 * distStar * distStar), .29);

        if(getTurretPosition() == PORT){
            extendPosition = Math.max((0.735 + -0.0247 * distPort + 3.31E-04 *distPort *distPort), .29);
        } else if(getTurretPosition() == STAR){
            extendPosition = Math.max((0.668 + -0.0195 * distStar + 2.54E-04 * distStar * distStar)+.0125, .29);
        } else {
            extendPosition = RETRACTED.getPosition();
        }

        /*if (gamepad1.y) {
            extendPosition = 0.2;
        } else if (gamepad1.b) {
            extendPosition = .15;
        } else if (gamepad1.a) {
            extendPosition = .125;
        } else if(gamepad1.x){
            extendPosition = .1;
        } else if(gamepad1.dpad_up){
            extendPosition = .05;
        } else if(gamepad1.dpad_right){
            extendPosition = 0;
        } else if(gamepad1.dpad_down){
            extendPosition = .1;
        } else if(gamepad1.dpad_left){
            extendPosition = 0;
        }*/
        return extendPosition;
    }
    public double getArmPower(){
        if(gamepad2.left_stick_y<0){
            armPower = 1;
            reference = (robot.armPort.getCurrentPosition()+robot.armStar.getCurrentPosition())/2.0;
        } else if(gamepad2.left_stick_y>0){
            armPower = -1;
            reference = (robot.armPort.getCurrentPosition()+robot.armStar.getCurrentPosition())/2.0;
        } else {
            double state = (robot.armPort.getCurrentPosition()+robot.armStar.getCurrentPosition())/2.0;
            armPower = armController.calculate(state, reference) + kg;
        }
        return armPower;
    }
}