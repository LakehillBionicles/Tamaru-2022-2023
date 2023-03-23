package org.firstinspires.ftc.teamcode.Tamaru2.BaseClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Tamaru2Hardware extends LinearOpMode{
    public DcMotorEx fpd = null;
    public DcMotorEx fsd = null;
    public DcMotorEx bpd = null;
    public DcMotorEx bsd = null;

    public DcMotorEx armPort = null;
    public DcMotorEx armStar = null;

    public Servo servoHand = null;
    public Servo servoTurret = null;
    public Servo servoExtend = null;
    public DcMotor extendEncoder = null;
    public Servo servoPoleToucherPort = null;
    public Servo servoPoleToucherStar = null;

    public ColorSensor colorSensorPort = null;
    public ColorSensor colorSensorFront = null;
    public ColorSensor colorSensorHand = null;

    public TouchSensor touchSensorPort = null;
    public TouchSensor touchSensorStar = null;

    public TouchSensor limitSwitch = null;

    public DistanceSensor distSensorHand = null;
    public DistanceSensor distSensorArm = null;
    public DistanceSensor distSensorPort = null;
    public DistanceSensor distSensorStar = null;
    public DistanceSensor distSensorFront = null;

    public RevBlinkinLedDriver lights = null;

    private ElapsedTime runtime = new ElapsedTime();

    public static final double handOpen = .75;//.8, .7
    public static final double handClosed = 0;

    public static final double turretForward = .55;
    public static final double turretPort = 1;
    public static final double turretStar = 0.05;

    static final double COUNTS_PER_MOTOR_REV_BE = 8192;
    static final double DRIVE_GEAR_REDUCTION_BE = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES_BE = 3.5;     // For figuring circumference
    public static final double COUNTS_PER_INCH_BE = (COUNTS_PER_MOTOR_REV_BE * DRIVE_GEAR_REDUCTION_BE) /
            (WHEEL_DIAMETER_INCHES_BE * 3.1415);

    public static final double maxVelocity = 2500;


    HardwareMap hwMap = null;
    /*private ElapsedTime time = new ElapsedTime();  */

    public Tamaru2Hardware(){
    }

    @Override

    public void runOpMode() {
    }

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        fpd = hwMap.get(DcMotorEx.class, "fpd");
        fsd = hwMap.get(DcMotorEx.class, "fsd");
        bpd = hwMap.get(DcMotorEx.class, "bpd");
        bsd = hwMap.get(DcMotorEx.class, "bsd");

        armPort = hwMap.get(DcMotorEx.class, "armPort");
        armStar = hwMap.get(DcMotorEx.class, "armStar");

        servoHand = hwMap.get(Servo.class, "servoHand");
        servoTurret = hwMap.get(Servo.class, "servoTurret");
        servoExtend = hwMap.get(Servo.class, "servoExtend");
        extendEncoder = hwMap.get(DcMotor.class, "extendEncoder");

        servoPoleToucherPort = hwMap.get(Servo.class, "servoPoleToucherPort");
        servoPoleToucherStar = hwMap.get(Servo.class, "servoPoleToucherStar");

        //colorSensorPort = hwMap.get(ColorSensor.class, "colorSensorPort");
        colorSensorFront = hwMap.get(ColorSensor.class, "colorSensorFront");
        //colorSensorHand = hwMap.get(ColorSensor.class, "colorSensorHand");

        distSensorHand = hwMap.get(DistanceSensor.class, "distSensorHand");
        distSensorPort = hwMap.get(DistanceSensor.class, "distSensorPort");
        distSensorStar = hwMap.get(DistanceSensor.class, "distSensorStar");
        //distSensorFront = hwMap.get(DistanceSensor.class, "distSensorFront");

        touchSensorPort = hwMap.get(TouchSensor.class, "touchSensorPort");
        touchSensorStar = hwMap.get(TouchSensor.class, "touchSensorStar");

        //limitSwitch = hwMap.get(TouchSensor.class, "limitSwitch");

        lights = hwMap.get(RevBlinkinLedDriver.class, "lights");

        //flipped 2/13 2:58 and again 2/13 3:02
        fpd.setDirection(DcMotorSimple.Direction.FORWARD);
        fsd.setDirection(DcMotorSimple.Direction.FORWARD);
        bpd.setDirection(DcMotorSimple.Direction.FORWARD);
        bsd.setDirection(DcMotorSimple.Direction.REVERSE);

        armPort.setDirection(DcMotorSimple.Direction.FORWARD);
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
