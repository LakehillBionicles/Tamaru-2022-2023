package org.firstinspires.ftc.teamcode.Tamaru3.Auto3;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor; //DcMotorEx?

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Tamaru3.Tamaru3Hardware;
/**
 * Class name: DashboardTestingAutoFunctions
 * Class Type: tele
 * Class Function: used to test auto function and PID loop tuning for drive and arm
 * Other Notes:
 */
@Config
@TeleOp (name = "auto dashboard testing", group = "dashboard")
public class DashboardTestingAutoFunctions extends LinearOpMode {
    Tamaru3Hardware robot = new Tamaru3Hardware();
    private PIDController yController;
    private PIDController xController;
    private PIDController thetaController;
    private PIDController armController;

    BNO055IMU imu;
    Orientation robotTheta;

    public static double yTarget = 0;
    public static double xTarget = 0;
    public static double thetaTarget = 0;
    public static double armTarget = 0;
    public static double turretTarget = 0.6;
    public static double extensionTarget = 0;
    public static double handTarget = 0;

    public static double py = 0.075, iy = 0.0005, dy = 0.01;
    public static double px = 0.25, ix = 0.001, dx = 0.05;
    public static double pTheta = 0.01, iTheta = 0.0, dTheta = 0.001;
    public static double pArm = 0, iArm = 0, dArm = 0;

    public static double xMultiplier = 1;

    double max = 2500;
    int loops = 1;

    public final double COUNTS_PER_ODO_REV = 8192;
    public final double ODO_GEAR_REDUCTION = (1.0); // This is < 1.0 if geared UP
    public final double ODO_WHEEL_DIAMETER_INCHES = 2.0;  // For figuring circumference
    public final double ODO_COUNTS_PER_INCH = ((COUNTS_PER_ODO_REV * ODO_GEAR_REDUCTION) /
            (ODO_WHEEL_DIAMETER_INCHES * 3.1415));

    public final double WHEEL_COUNTS_PER_INCH = 22.48958;

    public final double odoWheelGap = 12.5;

    public final int downArmTarget = 0;
    public final int lowPoleArmTarget = 2750;//2800
    public final int midPoleArmTarget = 4000;//3800, 3900
    public final int highPoleArmTarget = 5400;
    public final int fiveConeArmTarget = 1200;
    public final int fourConeArmTarget = 1000;
    public final int threeConeArmTarget = 850;
    public final int twoConeArmTarget = 650;

    public String color = "";

    public void runOpMode(){
        robot.init(hardwareMap);
        yController = new PIDController(py, iy, dy);
        xController = new PIDController(px, ix, dx);
        thetaController = new PIDController(pTheta, iTheta, dTheta);
        armController = new PIDController(pArm, iArm, dArm);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        robot.servoHand.setPosition(robot.handClosed);
        robot.servoExtend.setPosition(0);
        //robot.servoPoleToucherPort.setPosition(.63);
        //robot.servoPoleToucherStar.setPosition(.5);

        waitForStart();

        robot.fpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.fsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.fpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.fsd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bsd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.armPortI.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armPortO.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armStarI.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armStarO.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.armPortI.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.armPortO.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.armStarI.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.armStarO.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while(opModeIsActive()) {
            Control(yTarget, xTarget, thetaTarget, armTarget,  turretTarget, extensionTarget, handTarget, 10000, false, false, false);
            //PIDDriveControl(62, -2, 0, 5, false, false, false);
            //stop();
        }
    }

    /**
     * sets power and positions to drive motors, arm motors, turret servo, extension servo, and hand servo
     * @param targetY target front/back position in inches, positive is forward
     * @param targetX target strafe position in inches, positive is port
     * @param targetTheta target theta position in degrees, positive is clockwise
     * @param targetArm target arm position in encoder ticks
     * @param targetTurret target turret position
     * @param targetExtension target extension position
     * @param targetHand target hand position
     * @param timeout timeout, in seconds
     * @param portDist use port distance sensor to break the loop
     * @param starDist use star distance sensor to break the loop
     * @param handDist use hand distance sensor to break the loop
     */
    public void Control(double targetY, double targetX, double targetTheta, double targetArm, double targetTurret, double targetExtension, double targetHand, double timeout, boolean portDist, boolean starDist, boolean handDist){
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
        armController.setTolerance(5);

        int fpdPos = robot.fpd.getCurrentPosition();
        int bpdPos = robot.bpd.getCurrentPosition();
        int fsdPos = robot.fsd.getCurrentPosition();
        int bsdPos = robot.bsd.getCurrentPosition();
        int armStarPos = robot.armStarI.getCurrentPosition();
        int BOWPos = robot.armPortI.getCurrentPosition();

        double robotY = ((fpdPos+bpdPos+fsdPos+bsdPos)/4.0)/WHEEL_COUNTS_PER_INCH;
        robotTheta = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double robotX = (BOWPos / ODO_COUNTS_PER_INCH) - (2.5 * ((fpdPos+bpdPos+fsdPos+bsdPos)/4.0)/WHEEL_COUNTS_PER_INCH/odoWheelGap);

        resetRuntime();
        //while(((!yController.atSetPoint())||(!xController.atSetPoint())||(!thetaController.atSetPoint())||(!armController.atSetPoint()))&&(getRuntime()<timeout)) {
            fpdPos = robot.fpd.getCurrentPosition();
            bpdPos = robot.bpd.getCurrentPosition();
            fsdPos = robot.fsd.getCurrentPosition();
            bsdPos = robot.bsd.getCurrentPosition();
            armStarPos = robot.armStarI.getCurrentPosition();
            BOWPos = robot.armPortI.getCurrentPosition();

            robotY = ((fpdPos+bpdPos+fsdPos+bsdPos)/4.0)/WHEEL_COUNTS_PER_INCH;
            robotTheta = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            //robotX = -1*((BOWPos / ODO_COUNTS_PER_INCH) - (2.5 * ((fpdPos+bpdPos+fsdPos+bsdPos)/4.0)/WHEEL_COUNTS_PER_INCH/odoWheelGap));
            robotX = -1*((BOWPos / ODO_COUNTS_PER_INCH));

            double pidY = yController.calculate(robotY, yController.getSetPoint());
            double pidX = -xController.calculate(robotX, xController.getSetPoint());
            double pidTheta = -thetaController.calculate(robotTheta.firstAngle, thetaController.getSetPoint());
            double pidArm = armController.calculate(armStarPos, armController.getSetPoint());

            double vY = robot.maxVelocity * pidY;
            double vX = robot.maxVelocity * pidX;
            double vTheta = robot.maxVelocity * pidTheta;
            double powerArm = pidArm;

            robot.fpd.setVelocity((vY + xMultiplier*vX + vTheta));
            robot.bpd.setVelocity((vY - xMultiplier*vX + vTheta));
            robot.fsd.setVelocity((vY - xMultiplier*vX - vTheta));
            robot.bsd.setVelocity((vY + xMultiplier*vX - vTheta));

            robot.armPortI.setPower(powerArm);
            robot.armPortO.setPower(powerArm);
            robot.armStarI.setPower(powerArm);
            robot.armStarO.setPower(powerArm);

            robot.servoHand.setPosition(targetHand);
            robot.servoTurret.setPosition(targetTurret);
            robot.servoExtend.setPosition(targetExtension);

            /*if((portDist)&&(((robot.distSensorPort.getDistance(DistanceUnit.CM)<15)))){
                break;
            }
            if((starDist)&&(robot.distSensorStar.getDistance(DistanceUnit.CM)<15)){
                break;
            }
            if((handDist)&&(robot.distSensorPort.getDistance(DistanceUnit.CM)<4)){
                break;
            }*/

            loops++;
            telemetry.addData("loops", loops);
            telemetry.addData("robotArm", armStarPos);
            telemetry.addData("targetArm", targetArm);
            telemetry.update();
        /*}
        robot.fpd.setVelocity(0);
        robot.bpd.setVelocity(0);
        robot.fsd.setVelocity(0);
        robot.bsd.setVelocity(0);

        robot.armPortI.setPower(0);
        robot.armPortO.setPower(0);
        robot.armStarI.setPower(0);
        robot.armStarO.setPower(0);*/
    }

    /*public void PIDDriveControl(double targetY, double targetX, double targetTheta, double timeout, boolean portDist, boolean starDist, boolean handDist){
        yController.setPID(py, iy, dy);
        xController.setPID(px, ix, dx);
        thetaController.setPID(pTheta, iTheta, dTheta);

        yController.setSetPoint(targetY);
        xController.setSetPoint(targetX);
        thetaController.setSetPoint(Math.toRadians(targetTheta));

        yController.setTolerance(.1);
        xController.setTolerance(.1);
        thetaController.setTolerance(.25);


        int portAvg = (robot.fpd.getCurrentPosition() + robot.bpd.getCurrentPosition()) / 2;
        int starAvg = (robot.fsd.getCurrentPosition()+robot.bsd.getCurrentPosition())/2;
        double robotY = ((portAvg+starAvg)/2)/WHEEL_COUNTS_PER_INCH;
        robotTheta = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double robotX = (robot.BOW.getCurrentPosition() / ODO_COUNTS_PER_INCH) - (2.5 * (portAvg-starAvg)/WHEEL_COUNTS_PER_INCH/odoWheelGap);

        double pidY = 0;
        double pidX = 0;
        double pidTheta = 0;

        resetRuntime();
        while(((!yController.atSetPoint())||(!xController.atSetPoint())||(!thetaController.atSetPoint()))&&(getRuntime()<timeout)) {
            portAvg = (robot.fpd.getCurrentPosition() + robot.bpd.getCurrentPosition()) / 2;
            starAvg = (robot.fsd.getCurrentPosition()+robot.bsd.getCurrentPosition())/2;
            robotY = ((portAvg+starAvg)/2)/WHEEL_COUNTS_PER_INCH;
            robotTheta = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            robotX = (robot.BOW.getCurrentPosition() / ODO_COUNTS_PER_INCH) - (2.5 * (portAvg-starAvg)/WHEEL_COUNTS_PER_INCH/odoWheelGap);

            pidY = yController.calculate(robotY, targetY);
            pidX = xController.calculate(robotX, targetX);
            pidTheta = thetaController.calculate(robotTheta.firstAngle, targetTheta);

            double yVelocity = pidY * max;
            double xVelocity = -pidX * max;
            double thetaVelocity = -pidTheta * max;

            robot.fpd.setVelocity(yVelocity + 3*xVelocity + thetaVelocity);
            robot.bpd.setVelocity(yVelocity - 3*xVelocity + thetaVelocity);
            robot.fsd.setVelocity(yVelocity - 3*xVelocity - thetaVelocity);
            robot.bsd.setVelocity(yVelocity + 3*xVelocity - thetaVelocity);

            if((portDist)&&(((robot.distSensorPort.getDistance(DistanceUnit.CM)<15)))){
                break;
            }
            if((starDist)&&(robot.distSensorStar.getDistance(DistanceUnit.CM)<15)){
                break;
            }
            if((handDist)&&(robot.distSensorHand.getDistance(DistanceUnit.CM)<3)){
                break;
            }
        }
    }*/

    /*public void PIDArmControl(double targetArm, double timeout){
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
    }*/

}
