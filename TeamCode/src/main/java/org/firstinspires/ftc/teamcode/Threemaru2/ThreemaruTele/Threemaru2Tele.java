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
//import org.firstinspires.ftc.teamcode.Threemaru.ThreemaruHardware;
import org.firstinspires.ftc.teamcode.Threemaru2.Threemaru2Hardware;

@TeleOp
//@Disabled
public class Threemaru2Tele extends LinearOpMode {
    Threemaru2Hardware robot = new Threemaru2Hardware();
    public double drivePower, strafePower, rotPower, armPower;
    public double extendPosition = RETRACTED.getPosition();
    public int drivePowerDenom = 1;
    public HandSubsystem.HandPos handPos1 = OPEN1;
    public HandSubsystem.HandPos handPos2 = OPEN2;
    public TurretSubsystem.TurretPos turretPosition = FORWARD;
    private PIDController armController;
    public static double p = 0.001, i = 0, d = 0.0001, kg = 0.001;
    public static double reference = 0;
    public static double maxVelocity = 4000;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();
    public void runOpMode() {
        robot.init(hardwareMap);
        armController = new PIDController(p, i, d);

        robot.armPort.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armStar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorTurret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.armPort.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.armStar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorTurret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            robot.fpd.setPower(-gamepad2.left_stick_y + gamepad2.left_stick_x + gamepad2.right_stick_x);
            robot.bpd.setPower(-gamepad2.left_stick_y - gamepad2.left_stick_x + gamepad2.right_stick_x);
            robot.fsd.setPower(-gamepad2.left_stick_y - gamepad2.left_stick_x - gamepad2.right_stick_x);
            robot.bsd.setPower(-gamepad2.left_stick_y + gamepad2.left_stick_x - gamepad2.right_stick_x);

            robot.armStar.setPower(getArmPower());
            robot.armPort.setPower(getArmPower());

            robot.motorTurret.setPower(getTurretPower());

            robot.servoHand1.setPosition(getHandPos1().getPosition());
            robot.servoHand2.setPosition(getHandPos2().getPosition());
            //robot.servoExtend.setPosition(getExtendPosition());
            robot.servoExtend.setPosition(RETRACTED.getPosition());
            telemetry.addData("fsd", robot.fsd.getCurrentPosition());
            telemetry.addData("fpd", robot.fpd.getCurrentPosition());
            telemetry.addData("bsd", robot.bsd.getCurrentPosition());
            telemetry.addData("bpd", robot.bpd.getCurrentPosition());
            telemetry.update();

            //telemetry.addData("turret", robot.motorTurret.getCurrentPosition());
            //telemetry.update();
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

    public double getTurretPower(){
        double turretPower;
        if(gamepad2.dpad_right){
            turretPower = 0.3;
        } else if(gamepad2.dpad_left){
            turretPower = -0.3;
        } else {
            turretPower = 0;
        }

        return turretPower;
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
        double distPort = (robot.distSensorPort.getDistance(DistanceUnit.CM));
        double distStar = robot.distSensorStar.getDistance(DistanceUnit.CM);
        //extendPosition = Math.max(1.92 + -0.126 * distPort + 2.23E-03 * distPort * distPort, 0);

        if(getTurretPosition() == PORT){
            extendPosition = Math.max(0.491 - (0.0241*distPort) + (3.25E-04*distPort*distPort), 0);
        } else if(getTurretPosition() == STAR){
            extendPosition = Math.max(0.557 - (0.0367*distStar) + (6.33E-04*distStar*distStar), 0);
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
        if(gamepad2.dpad_up){
            armPower = 1;
            reference = (robot.armPort.getCurrentPosition()+robot.armStar.getCurrentPosition())/2.0;
        } else if(gamepad2.dpad_down){
            armPower = -1;
            reference = (robot.armPort.getCurrentPosition()+robot.armStar.getCurrentPosition())/2.0;
        } else {
            double state = (robot.armPort.getCurrentPosition()+robot.armStar.getCurrentPosition())/2.0;
            armPower = armController.calculate(state, reference) + kg;
        }
        return armPower;
    }
}