package org.firstinspires.ftc.teamcode.Tamaru2.Auto2.CurrentPaths;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor; //DcMotorEx?

import org.firstinspires.ftc.teamcode.Tamaru2.BaseClasses.Tamaru2Hardware;


@Config
@Autonomous
public class justParking extends LinearOpMode {
    Tamaru2Hardware robot = new Tamaru2Hardware();
    private PIDController yController;
    private PIDController xController;
    private PIDController thetaController;
    private PIDController armController;

    public static double py = 0.0275, iy = 0.00055, dy = 0;
    public static double fy = 0;
    public static double px = 0.075, ix = 0.0001, dx = 0;
    public static double fx = 0;
    public static double pTheta = 0.855, iTheta = 0.01, dTheta = 0;
    public static double fTheta = 0.05;
    public static double pArm = .03, iArm = 0.001, dArm = 0.0005;
    public static double fArm = 0;


    public final double COUNTS_PER_ODO_REV = 8192;
    public final double ODO_GEAR_REDUCTION = (1.0); // This is < 1.0 if geared UP
    public final double ODO_WHEEL_DIAMETER_INCHES = 2.0;  // For figuring circumference
    public final double ODO_COUNTS_PER_INCH = ((COUNTS_PER_ODO_REV * ODO_GEAR_REDUCTION) /
            (ODO_WHEEL_DIAMETER_INCHES * 3.1415));

    public final double odoWheelGap = 12.5;//used to be 11.5

    public final int downArmTarget = 0;
    public final int lowPoleArmTarget = 2750;//2800
    public final int midPoleArmTarget = 3900;//3800
    public final int highPoleArmTarget = 5400;//this one doesn't work, not enough power to get up all the way
    public final int fiveConeArmTarget = 1100;
    public final int fourConeArmTarget = 1000;
    public final int threeConeArmTarget = 850;
    public final int twoConeArmTarget = 650;

    public String color = "";


    public void runOpMode() {
        robot.init(hardwareMap);
        yController = new PIDController(py, iy, dy);
        xController = new PIDController(px, ix, dx);
        thetaController = new PIDController(pTheta, iTheta, dTheta);
        armController = new PIDController(pArm, iArm, dArm);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.servoHand.setPosition(robot.handClosed);
        robot.servoExtend.setPosition(0);
        robot.servoPoleToucherPort.setPosition(.63);
        robot.servoPoleToucherStar.setPosition(.5);


        waitForStart();

        robot.fpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.fsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.fpd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.bpd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.fsd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.bsd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.armPort.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armStar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.armPort.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.armStar.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive()) {
            Control(30, 0, 0, lowPoleArmTarget, 0, 0, robot.handClosed, 1);
            if(senseColorsStar().equals("green")){
                color = "green";
                telemetry.addData("color", color);
                telemetry.update();
            }else if(senseColorsStar().equals("blue")){
                color = "blue";
                telemetry.addData("color", color);
                telemetry.update();
            }else if(senseColorsStar().equals("red")){
                color = "red";
                telemetry.addData("color", color);
                telemetry.update();
            }
            PIDDriveControl(60, 0, 0, 1);
            PIDDriveControl(50, 0, 0, 1.5);
            //Control(60, 0, 0, lowPoleArmTarget, 0, 0, robot.handClosed, 1);
            //Control(54.5, 0, 0, lowPoleArmTarget, 0, 0, robot.handClosed, 1);

            if(color=="green"){
                PIDDriveControl(50, 0, -90, 1);
                PIDDriveControl(27, 0, -90, 1);
            }else if(color=="blue"){
                PIDDriveControl(44, 0, 0, 1);
            }else{
                PIDDriveControl(50, 0, -90, 1);
                PIDDriveControl(83, 0, -90, 1);
            }

            stop();
        }
    }

    public void Control(double targetY, double targetX, double targetTheta, double targetArm, double targetTurret, double targetExtension, double targetHand, double timeout){
        yController.setPID(py, iy, dy);
        xController.setPID(px, ix, dx);
        thetaController.setPID(pTheta, iTheta, dTheta);
        armController.setPID(pArm, iArm, dArm);

        yController.setSetPoint(targetY);
        xController.setSetPoint(targetX);
        thetaController.setSetPoint(Math.toRadians(targetTheta));
        armController.setSetPoint(targetArm);

        yController.setTolerance(.5);
        xController.setTolerance(.1);
        thetaController.setTolerance(.25);
        armController.setTolerance(20);

        double robotY = ((robot.fpd.getCurrentPosition() + robot.fsd.getCurrentPosition()) / 2) / ODO_COUNTS_PER_INCH;
        double robotTheta = (robot.fpd.getCurrentPosition() - robot.fsd.getCurrentPosition()) / ODO_COUNTS_PER_INCH / odoWheelGap;
        double robotX = (robot.bpd.getCurrentPosition() / ODO_COUNTS_PER_INCH) - (2.5 * robotTheta);
        double robotArmPort = robot.armPort.getCurrentPosition();
        double robotArmStar = robot.armStar.getCurrentPosition();
        double robotArm = (robotArmPort+robotArmStar)/2;

        resetRuntime();
        while(((!yController.atSetPoint())||(!xController.atSetPoint())||(!thetaController.atSetPoint())||(!armController.atSetPoint()))&&(getRuntime()<timeout)) {
            robotY = ((robot.fpd.getCurrentPosition() + robot.fsd.getCurrentPosition()) / 2) / ODO_COUNTS_PER_INCH;
            robotTheta = (robot.fpd.getCurrentPosition() - robot.fsd.getCurrentPosition()) / ODO_COUNTS_PER_INCH / odoWheelGap;
            robotX = (robot.bpd.getCurrentPosition() / ODO_COUNTS_PER_INCH) - (2.5 * robotTheta);
            robotArmPort = robot.armPort.getCurrentPosition();
            robotArmStar = robot.armStar.getCurrentPosition();
            robotArm = (robotArmPort+robotArmStar)/2;

            double pidY = yController.calculate(robotY, yController.getSetPoint());
            double pidX = xController.calculate(robotX, xController.getSetPoint());
            double pidTheta = thetaController.calculate(robotTheta, thetaController.getSetPoint());
            double pidArm = armController.calculate(robotArm, armController.getSetPoint());

            double ffY = (targetY / ODO_COUNTS_PER_INCH) * fy;
            double ffX = (targetX / ODO_COUNTS_PER_INCH) * fx;
            double ffTheta = (targetTheta / ODO_COUNTS_PER_INCH) * fTheta;
            double ffArm = targetArm * fArm;

            double yPower = -(pidY + ffY);
            double xPower = -(pidX + ffX);
            double thetaPower = -(pidTheta + ffTheta);
            double armPower = (pidArm + ffArm);

            robot.fpd.setPower(yPower - xPower + thetaPower);
            robot.bpd.setPower(yPower + xPower + thetaPower);
            robot.fsd.setPower(yPower + xPower - thetaPower);
            robot.bsd.setPower(yPower - xPower - thetaPower);

            robot.armPort.setPower(armPower);
            robot.armStar.setPower(armPower);

            robot.servoHand.setPosition(targetHand);
            robot.servoTurret.setPosition(targetTurret);
            robot.servoExtend.setPosition(targetExtension);
        }
        robot.armPort.setPower(0);
        robot.armStar.setPower(0);
    }

    public void PIDDriveControl(double targetY, double targetX, double targetTheta, double timeout){
        yController.setPID(py, iy, dy);
        xController.setPID(px, ix, dx);
        thetaController.setPID(pTheta, iTheta, dTheta);

        yController.setSetPoint(targetY);
        xController.setSetPoint(targetX);
        thetaController.setSetPoint(Math.toRadians(targetTheta));

        yController.setTolerance(.1);
        xController.setTolerance(.1);
        thetaController.setTolerance(.25);

        double robotY = ((robot.fpd.getCurrentPosition() + robot.fsd.getCurrentPosition()) / 2) / ODO_COUNTS_PER_INCH;
        double robotTheta = (robot.fpd.getCurrentPosition() - robot.fsd.getCurrentPosition()) / ODO_COUNTS_PER_INCH / odoWheelGap;
        double robotX = (robot.bpd.getCurrentPosition() / ODO_COUNTS_PER_INCH) - (2.5 * robotTheta);

        resetRuntime();
        while(((!yController.atSetPoint())||(!xController.atSetPoint())||(!thetaController.atSetPoint()))&&(getRuntime()<timeout)) {
            robotY = ((robot.fpd.getCurrentPosition() + robot.fsd.getCurrentPosition()) / 2) / ODO_COUNTS_PER_INCH;
            robotTheta = (robot.fpd.getCurrentPosition() - robot.fsd.getCurrentPosition()) / ODO_COUNTS_PER_INCH / odoWheelGap;
            robotX = (robot.bpd.getCurrentPosition() / ODO_COUNTS_PER_INCH) - (2.5 * robotTheta);

            double pidY = yController.calculate(robotY, yController.getSetPoint());
            double pidX = xController.calculate(robotX, xController.getSetPoint());
            double pidTheta = thetaController.calculate(robotTheta, thetaController.getSetPoint());

            double ffY = (targetY / ODO_COUNTS_PER_INCH) * fy;
            double ffX = (targetX / ODO_COUNTS_PER_INCH) * fx;
            double ffTheta = (targetTheta / ODO_COUNTS_PER_INCH) * fTheta;

            double yPower = -(pidY + ffY);
            double xPower = -(pidX + ffX);
            double thetaPower = -(pidTheta + ffTheta);

            robot.fpd.setPower(yPower - xPower + thetaPower);
            robot.bpd.setPower(yPower + xPower + thetaPower);
            robot.fsd.setPower(yPower + xPower - thetaPower);
            robot.bsd.setPower(yPower - xPower - thetaPower);
        }
    }

    public void PIDArmControl(double targetArm, double timeout){
        armController.setPID(pArm, iArm, dArm);
        armController.setSetPoint(targetArm);
        armController.setTolerance(20);

        double robotArmPort = robot.armPort.getCurrentPosition();
        double robotArmStar = robot.armStar.getCurrentPosition();
        double robotArm = (robotArmPort+robotArmStar)/2;

        resetRuntime();
        while((!armController.atSetPoint()) && getRuntime()<timeout){
            robotArmPort = robot.armPort.getCurrentPosition();
            robotArmStar = robot.armStar.getCurrentPosition();
            robotArm = (robotArmPort+robotArmStar)/2;
            double pidArm = armController.calculate(robotArm, armController.getSetPoint());
            double ffArm = targetArm * fArm;
            double armPower = (pidArm + ffArm);
            robot.armPort.setPower(armPower);
            robot.armStar.setPower(armPower);
        }
        robot.armPort.setPower(0);
        robot.armStar.setPower(0);
    }

    public String senseColorsStar () {
        String colorStar = "blank";

        while (opModeIsActive() && colorStar.equals("blank")) {
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

