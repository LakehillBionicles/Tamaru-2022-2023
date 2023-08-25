package org.firstinspires.ftc.teamcode.Threemaru.Tele4;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Threemaru.ThreemaruHardware;

//@Disabled
@Config
@TeleOp
public class ServoLimits extends OpMode {
    ThreemaruHardware robot = new ThreemaruHardware();

    public static double hand1Pos = 0.2;
    public static double hand2Pos = 0.9;
    public static double extendPos = 0.61;
    public static int turretPos = 0;
    public static double armPos = 0;
    public static double turretPower = 0;

    public static double maxVelocity = 4000;

    private PIDController turretController, armController;

    public static double pTurret = 0.005, iTurret = 0, dTurret = 0.00005;
    public static double pArm = 0.001, iArm = 0, dArm = 0.0001, gArm = 0.001;


    @Override
    public void init(){
        robot.init(hardwareMap);
        robot.motorTurret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //turretController = new PIDController(pTurret, iTurret, dTurret);
        armController = new PIDController(pArm, iArm, dArm);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.armPort.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armStar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorTurret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armPort.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.armStar.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.motorTurret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop(){
        robot.servoHand1.setPosition(hand1Pos);
        robot.servoHand2.setPosition(hand2Pos);
        robot.servoExtend.setPosition(extendPos);
        //PIDTurret(turretPos);

        armController.setPID(pArm, iArm, dArm);
        //turretController.setPID(pTurret, iTurret, dTurret);

        double robotArm = (robot.armPort.getCurrentPosition()+robot.armStar.getCurrentPosition())/2.0;
        //double robotTurret = robot.motorTurret.getCurrentPosition();
        double pidArm = armController.calculate(robotArm, armPos) + gArm;
        //double pidTurret = turretController.calculate(robotTurret, turretPos);
        double velocityArm = pidArm * maxVelocity;
        ///double velocityTurret = pidTurret * maxVelocity;
        if(gamepad1.dpad_right){
            robot.motorTurret.setPower(.1);
        } else if(gamepad1.dpad_left){
            robot.motorTurret.setPower(-.1);
        } else {
            robot.motorTurret.setPower(0);
        }

        //robot.armPort.setVelocity(velocityArm);
        //robot.armStar.setVelocity(velocityArm);
        //robot.motorTurret.setVelocity(velocityTurret);
        //encoderTurret(500);
        //turretTimeBased(-1, 3);
        //stop();
        //telemetry.addData("hand1Pos", hand1Pos);
        //telemetry.addData("hand2Pos", hand2Pos);
        telemetry.addData("extendPos", extendPos);
        telemetry.addData("Turret Position", robot.motorTurret.getCurrentPosition());
        telemetry.addData("distStar", robot.distSensorStar.getDistance(DistanceUnit.CM));
        telemetry.addData("distPort", robot.distSensorPort.getDistance(DistanceUnit.CM));
        //telemetry.addData("turretPos", turretPos);
        telemetry.update();
    }

    public void encoderTurret(int target){
        resetRuntime();
        double state = robot.motorTurret.getCurrentPosition();
        while((state < (target-20))){
            robot.motorTurret.setPower(.08);
            state = robot.motorTurret.getCurrentPosition();
        }
        while((state > (target+20))){
            robot.motorTurret.setPower(-.08);
            state = robot.motorTurret.getCurrentPosition();
        }
        robot.motorTurret.setPower(0);
    }
    public void turretTimeBased(double direction, double timeout){
        resetRuntime();
        while(getRuntime()<timeout){
            robot.motorTurret.setPower(direction * .05);
        }
        robot.motorTurret.setPower(0);
    }
    public void armToPosition(int position){
        robot.armPort.setTargetPosition(position);
        robot.armStar.setTargetPosition(position);

        robot.armPort.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armStar.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.armPort.setPower(1);
        robot.armStar.setPower(1);
    }

    public void PIDTurret(double target) {
        turretController.setPID(pTurret, iTurret, dTurret);

        turretController.setSetPoint(target);

        turretController.setTolerance(.1);

        double turretPos = robot.motorTurret.getCurrentPosition();

        resetRuntime();
        while (((!turretController.atSetPoint()))) {
            turretPos = robot.motorTurret.getCurrentPosition();

            double pid = turretController.calculate(turretPos, turretController.getSetPoint());
            double velocityY = pid * maxVelocity;

            robot.motorTurret.setVelocity(velocityY);
        }
    }

    public void PIDArm(double target) {
        armController.setPID(pArm, iArm, dArm);

        armController.setSetPoint(target);

        armController.setTolerance(.1);

        double state = (robot.armPort.getCurrentPosition()+robot.armStar.getCurrentPosition())/2.0;

        resetRuntime();
        while (((!armController.atSetPoint()))) {

            state = (robot.armPort.getCurrentPosition()+robot.armStar.getCurrentPosition())/2.0;
            double pid = armController.calculate(state, target) + gArm;
            double velocity = pid * maxVelocity;

            robot.armPort.setVelocity(velocity);
            robot.armStar.setVelocity(velocity);
        }
    }
}