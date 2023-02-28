package org.firstinspires.ftc.teamcode.Tamaru3.TeleOp3.PIDTuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor; //DcMotorEx?

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Tamaru3.Tamaru3Hardware;

@Config
@Autonomous
public class PIDTuningAuto extends OpMode {
    Tamaru3Hardware robot = new Tamaru3Hardware();
    private PIDController yController;
    private PIDController xController;
    private PIDController thetaController;
    private PIDController armController;

    public static double py = 0.05, iy = 0.0001, dy = 0.015;
    public static double px = 0.1, ix = 0.001, dx = 0.025;
    public static double pTheta = 0.05, iTheta = 0.0, dTheta = 0.05;
    public static double pArm = .03, iArm = 0.001, dArm = 0.0005;

    public static double vMax = 2500;

    int loops = 0;
    int vDenom = 1;

    public final double COUNTS_PER_ODO_REV = 8192;
    public final double ODO_GEAR_REDUCTION = (1.0); // This is < 1.0 if geared UP
    public final double ODO_WHEEL_DIAMETER_INCHES = 2.0;  // For figuring circumference
    public final double ODO_COUNTS_PER_INCH = ((COUNTS_PER_ODO_REV * ODO_GEAR_REDUCTION) /
            (ODO_WHEEL_DIAMETER_INCHES * 3.1415));

    public final double odoWheelGap = 12.5;//used to be 11.5

    public final int downArmTarget = 0;
    public final int lowPoleArmTarget = 2750;//2800
    public final int midPoleArmTarget = 4000;//3800, 3900
    public final int highPoleArmTarget = 5400;//this one doesn't work, not enough power to get up all the way
    public final int fiveConeArmTarget = 1200;
    public final int fourConeArmTarget = 1000;
    public final int threeConeArmTarget = 850;
    public final int twoConeArmTarget = 650;

    public String color = "";

    @Override
    public void init() {
        robot.init(hardwareMap);
        yController = new PIDController(py, iy, dy);
        xController = new PIDController(px, ix, dx);
        thetaController = new PIDController(pTheta, iTheta, dTheta);
        armController = new PIDController(pArm, iArm, dArm);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.fpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.fsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.fpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.fsd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bsd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.BOW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.SOW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armPort_POW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.BOW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.SOW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.armPort_POW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.armStar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armStar.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.servoHand.setPosition(robot.handClosed);
        robot.servoExtend.setPosition(0);
    }

    @Override
    public void start() {
        PIDDriveControl(15, 0, 0, 1, false, false, false);
        PIDDriveControl(30, 0, 0, 1, false, false, false);
        //Drive forward to middle pole (twice for accuracy)
        //PIDDriveControl(62, 0, 0, 5, false, false, false);
        //Control(30, 0, 0, midPoleArmTarget, robot.turretForward, 0, robot.handClosed, 2, false, false, false);//1.5
        //Control(62, 0, 0, midPoleArmTarget, robot.turretForward, 0, robot.handClosed, 2, false, false, false);//1.5
        if (senseColorsStar().equals("green")) {
            color = "green";
            telemetry.addData("color", color);
            telemetry.update();
        } else if (senseColorsStar().equals("blue")) {
            color = "blue";
            telemetry.addData("color", color);
            telemetry.update();
        } else if (senseColorsStar().equals("red")) {
            color = "red";
            telemetry.addData("color", color);
            telemetry.update();
        }
            /*Control(40, 0, 0, midPoleArmTarget, robot.turretForward, 0, robot.handClosed, 5, false, true, false);//1
            //set extension and turret over pole
            robot.servoExtend.setPosition(.4);//.25
            robot.servoTurret.setPosition(robot.turretStar);
            try { wait(1500); } catch (InterruptedException e) {}
            //open hand
            robot.servoHand.setPosition(robot.handOpen);
            sleep(1000);
            //reset turret and extension
            robot.servoTurret.setPosition(robot.turretForward);
            robot.servoExtend.setPosition(0);
            sleep(1500);
            PIDArmControl(0, 5);
            //drive to stack and set arm height
            Control(56, 0, 0, fiveConeArmTarget, robot.turretForward, 0, robot.handOpen, 2, false, false, false);//2
            PIDDriveControl(56, 0, -90, 1.5, false, false, false);
            PIDDriveControl(83, 0, -90, 1.5, false, false, true);
            //grab top cone
            robot.servoHand.setPosition((robot.handClosed));
            sleep(500);
            PIDArmControl(lowPoleArmTarget, 1);
            //score top cone, 66.5, 67.5, 67
            PIDDriveControl(68, 0, -90, 1.5, true, false, false);//67.5
            robot.servoTurret.setPosition(robot.turretPort);
            robot.servoExtend.setPosition(.65);
            sleep(1500);
            robot.servoHand.setPosition(robot.handOpen);
            sleep(1000);
            //reset and drive to second cone
            robot.servoTurret.setPosition(robot.turretForward);
            robot.servoExtend.setPosition(0);
            sleep(1500);
            PIDArmControl(fourConeArmTarget, 1);
            //Control(84, 0, -90, 1.5, fourConeArmTarget, 0, robot.handOpen, 1.5);
            PIDDriveControl(83, 0, -90, 1.5, false, false, true);
            //grab second cone
            robot.servoHand.setPosition(robot.handClosed);
            sleep(500);
            PIDArmControl(lowPoleArmTarget, 1);
            //score second cone, 66.5
            PIDDriveControl(68, 0, -90, 1.5, true, false, false);
            robot.servoTurret.setPosition(robot.turretPort);
            robot.servoExtend.setPosition(.65);
            sleep(1500);
            robot.servoHand.setPosition(robot.handOpen);
            sleep(1000);
            //reset
            robot.servoTurret.setPosition(robot.turretForward);
            robot.servoExtend.setPosition(0);
            robot.servoHand.setPosition(robot.handClosed);
            if(color=="green"){
                PIDDriveControl(35, 0, -90,1, false, false, false);
                PIDArmControl(0, 2);
            }else if(color=="blue"){
                PIDDriveControl(53, 0, -90, 1, false, false, false);
                PIDArmControl(0, 2);
            }else{
                PIDDriveControl(83, 0, -90, 1, false, false, false);
            }

            double portDist = robot.distSensorPort.getDistance(DistanceUnit.CM);
            double starDist = robot.distSensorStar.getDistance(DistanceUnit.CM);

            telemetry.addData("portDist", portDist);
            telemetry.addData("starDist", starDist);
            telemetry.update();*/
        stop();
    }

    @Override
    public void loop(){

    }

    public void Control(double targetY, double targetX, double targetTheta, double targetArm, double targetTurret, double targetExtension, double targetHand, double timeout, boolean portDist, boolean starDist, boolean handDist){
        double POWlocation = robot.armPort_POW.getCurrentPosition();
        double SOWlocation = robot.SOW.getCurrentPosition();
        double BOWlocation = robot.BOW.getCurrentPosition();

        yController.setPID(py, iy, dy);
        xController.setPID(px, ix, dx);
        thetaController.setPID(pTheta, iTheta, dTheta);
        armController.setPID(pArm, iArm, dArm);

        yController.setSetPoint(targetY);
        xController.setSetPoint(targetX);
        thetaController.setSetPoint(Math.toRadians(targetTheta));
        armController.setSetPoint(targetArm);

        yController.setTolerance(.5);
        xController.setTolerance(.5);
        thetaController.setTolerance(.25);
        armController.setTolerance(20);

        double robotY = ((POWlocation + SOWlocation) / 2) / ODO_COUNTS_PER_INCH;
        double robotTheta = (POWlocation - SOWlocation) / ODO_COUNTS_PER_INCH / odoWheelGap;
        double robotX = (BOWlocation / ODO_COUNTS_PER_INCH) - (2.5 * robotTheta);
        double robotArmStar = robot.armStar.getCurrentPosition();

        resetRuntime();
        while(((!yController.atSetPoint())||(!xController.atSetPoint())||(!thetaController.atSetPoint())||(!armController.atSetPoint()))&&(getRuntime()<timeout)) {
            POWlocation = robot.armPort_POW.getCurrentPosition();
            SOWlocation = robot.SOW.getCurrentPosition();
            BOWlocation = robot.BOW.getCurrentPosition();

            robotY = ((POWlocation + SOWlocation) / 2) / ODO_COUNTS_PER_INCH;
            robotTheta = (POWlocation - SOWlocation) / ODO_COUNTS_PER_INCH / odoWheelGap;
            robotX = (BOWlocation / ODO_COUNTS_PER_INCH) - (2.5 * robotTheta);
            robotArmStar = robot.armStar.getCurrentPosition();

            double pidY = -yController.calculate(robotY, yController.getSetPoint());
            double pidX = -xController.calculate(robotX, xController.getSetPoint());
            double pidTheta = -thetaController.calculate(robotTheta, thetaController.getSetPoint());
            double pidArm = armController.calculate(robotArmStar, armController.getSetPoint());

            double vY = robot.maxVelocity * pidY;
            double vX = robot.maxVelocity * pidX;
            double vTheta = robot.maxVelocity * pidTheta;
            double powerArm = pidArm;

            vDenom = 4;

            robot.fpd.setVelocity((vY - vX + 5*vTheta)/vDenom);
            robot.bpd.setVelocity((vY + vX + 5*vTheta)/vDenom);
            robot.fsd.setVelocity((vY + vX - 5*vTheta)/vDenom);
            robot.bsd.setVelocity((vY - vX - 5*vTheta)/vDenom);

            robot.armPort_POW.setPower(powerArm);
            robot.armStar.setPower(powerArm);

            robot.servoHand.setPosition(targetHand);
            robot.servoTurret.setPosition(targetTurret);
            robot.servoExtend.setPosition(targetExtension);

            if((portDist)&&(((robot.distSensorPort.getDistance(DistanceUnit.CM)<15)))){
                break;
            }
            if((starDist)&&(robot.distSensorStar.getDistance(DistanceUnit.CM)<15)){
                break;
            }
            if((handDist)&&(robot.distSensorPort.getDistance(DistanceUnit.CM)<4)){
                break;
            }

            loops++;
        }
        robot.fpd.setVelocity(0);
        robot.bpd.setVelocity(0);
        robot.fsd.setVelocity(0);
        robot.bsd.setVelocity(0);

        robot.armPort_POW.setPower(0);
        robot.armStar.setPower(0);
    }

    public void PIDDriveControl(double targetY, double targetX, double targetTheta, double timeout, boolean portDist, boolean starDist, boolean handDist){
        yController.setPID(py, iy, dy);
        xController.setPID(px, ix, dx);
        thetaController.setPID(pTheta, iTheta, dTheta);

        yController.setSetPoint(targetY);
        xController.setSetPoint(targetX);
        thetaController.setSetPoint(Math.toRadians(targetTheta));

        yController.setTolerance(.1);
        xController.setTolerance(.1);
        thetaController.setTolerance(.25);

        double robotY = ((robot.armPort_POW.getCurrentPosition() + robot.SOW.getCurrentPosition()) / 2) / ODO_COUNTS_PER_INCH;
        double robotTheta = (robot.armPort_POW.getCurrentPosition() - robot.SOW.getCurrentPosition()) / ODO_COUNTS_PER_INCH / odoWheelGap;
        double robotX = (robot.BOW.getCurrentPosition() / ODO_COUNTS_PER_INCH) - (2.5 * robotTheta);

        resetRuntime();
        while(((!yController.atSetPoint())||(!xController.atSetPoint())||(!thetaController.atSetPoint()))&&(getRuntime()<timeout)) {
            robotY = ((robot.armPort_POW.getCurrentPosition() + robot.SOW.getCurrentPosition()) / 2) / ODO_COUNTS_PER_INCH;
            robotTheta = (robot.armPort_POW.getCurrentPosition() - robot.SOW.getCurrentPosition()) / ODO_COUNTS_PER_INCH / odoWheelGap;
            robotX = (robot.BOW.getCurrentPosition() / ODO_COUNTS_PER_INCH) - (2.5 * robotTheta);

            double pidY = -yController.calculate(robotY, yController.getSetPoint());
            double pidX = -xController.calculate(robotX, xController.getSetPoint());
            double pidTheta = -thetaController.calculate(robotTheta, thetaController.getSetPoint());

            double vY = vMax * pidY;
            double vX = vMax * pidX;
            double vTheta = vMax * pidTheta;

            robot.fpd.setVelocity(vY - vX + vTheta);
            robot.bpd.setVelocity(vY + vX + vTheta);
            robot.fsd.setVelocity(vY + vX - vTheta);
            robot.bsd.setVelocity(vY - vX - vTheta);

            if((portDist)&&(((robot.distSensorPort.getDistance(DistanceUnit.CM)<15)))){
                break;
            }
            if((starDist)&&(robot.distSensorStar.getDistance(DistanceUnit.CM)<15)){
                break;
            }
            if((handDist)&&(robot.distSensorHand.getDistance(DistanceUnit.CM)<3)){
                break;
            }

            telemetry.addData("yVelocity", vY);
            telemetry.update();
        }
        robot.fpd.setVelocity(0);
        robot.bpd.setVelocity(0);
        robot.fsd.setVelocity(0);
        robot.bsd.setVelocity(0);
    }

    public void PIDArmControl(double targetArm, double timeout){
        armController.setPID(pArm, iArm, dArm);
        armController.setSetPoint(targetArm);
        armController.setTolerance(20);

        double robotArmStar = robot.armStar.getCurrentPosition();

        resetRuntime();
        while((!armController.atSetPoint()) && getRuntime()<timeout){
            robotArmStar = robot.armStar.getCurrentPosition();

            double pidArm = armController.calculate(robotArmStar, armController.getSetPoint());

            double powerArm = pidArm;

            robot.armPort_POW.setPower(powerArm);
            robot.armStar.setPower(powerArm);

        }
        robot.armPort_POW.setPower(0);
        robot.armStar.setPower(0);
    }


    public String senseColorsStar () {
        String colorStar = "blank";

        while (colorStar.equals("blank")) {
            if (robot.colorSensorStar.red() > ((robot.colorSensorStar.blue())-5) && robot.colorSensorStar.red() > ((robot.colorSensorStar.green()))-20) {
                colorStar = "red";
                telemetry.addData("i see red", " ");
                telemetry.update();
                colorStar = "red";
                //sleeveColor.equals(red);

            } else if (robot.colorSensorStar.blue() > (robot.colorSensorStar.red()) && robot.colorSensorStar.blue() > (robot.colorSensorStar.green())) {
                colorStar = "blue";
                telemetry.addData("i see blue", " ");
                telemetry.update();
                colorStar = "blue";

            } else if (robot.colorSensorStar.green() > (robot.colorSensorStar.red()) && robot.colorSensorStar.green() > (robot.colorSensorStar.blue())) {
                colorStar = "green";
                telemetry.addData("i see green", " ");
                telemetry.update();
                colorStar = "green";

            } else {
                telemetry.addData("i see nothing", " ");
                telemetry.update();
                colorStar = "no go";

            }

        }
        return colorStar;
    }

}