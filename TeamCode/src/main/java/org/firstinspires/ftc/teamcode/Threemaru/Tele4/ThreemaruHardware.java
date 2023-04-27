package org.firstinspires.ftc.teamcode.Threemaru.Tele4;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ThreemaruHardware extends LinearOpMode{
    public DcMotorEx fpd = null;
    public DcMotorEx fsd = null;
    public DcMotorEx bpd = null;//POW
    public DcMotorEx bsd = null;//BOW

    public DcMotorEx armPort = null, armStar = null;
    public Servo servoHand1 = null, servoHand2 = null;
    public Servo servoTurret = null;
    public Servo servoExtend = null;

    public DistanceSensor distSensorPort = null;
    public DistanceSensor distSensorStar = null;

    public static final double COUNTS_PER_MOTOR_REV = 28, DRIVE_GEAR_REDUCTION = 1/18.8803, WHEEL_DIAMETER_INCHES = 1.8898*2;
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    public ElapsedTime runtime = new ElapsedTime();

    HardwareMap hwMap = null;

    public ThreemaruHardware(){
    }

    @Override
    public void runOpMode() {}

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        fpd = hwMap.get(DcMotorEx.class, "fpd");
        fsd = hwMap.get(DcMotorEx.class, "fsd");
        bpd = hwMap.get(DcMotorEx.class, "bpd");
        bsd = hwMap.get(DcMotorEx.class, "bsd");

        armPort = hwMap.get(DcMotorEx.class, "armPort");
        armStar = hwMap.get(DcMotorEx.class, "armStar");

        servoHand1 = hwMap.get(Servo.class, "servoHand1");
        servoHand2 = hwMap.get(Servo.class, "servoHand2");
        servoTurret = hwMap.get(Servo.class, "servoTurret");
        servoExtend = hwMap.get(Servo.class, "servoExtend");

        distSensorPort = hwMap.get(DistanceSensor.class, "distSensorPort");
        distSensorStar = hwMap.get(DistanceSensor.class, "distSensorStar");

        fpd.setDirection(DcMotorSimple.Direction.REVERSE);
        fsd.setDirection(DcMotorSimple.Direction.REVERSE);
        bpd.setDirection(DcMotorSimple.Direction.FORWARD);
        bsd.setDirection(DcMotorSimple.Direction.REVERSE);
        armPort.setDirection(DcMotorSimple.Direction.REVERSE);
        armStar.setDirection(DcMotorSimple.Direction.FORWARD);

        fpd.setPower(0);
        fsd.setPower(0);
        bpd.setPower(0);
        bsd.setPower(0);

        armPort.setPower(0);
        armStar.setPower(0);

        fpd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fsd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bpd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bsd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armPort.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armStar.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fpd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fsd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bpd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bsd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armPort.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armStar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}