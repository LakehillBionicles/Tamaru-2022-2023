package org.firstinspires.ftc.teamcode.Tamaru3;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Tamaru3Hardware extends LinearOpMode{
    public DcMotorEx fpd = null;
    public DcMotorEx fsd = null;
    public DcMotorEx bpd = null;
    public DcMotorEx bsd = null;

    public DcMotorEx BOW = null;
    public DcMotorEx SOW = null;

    public DcMotorEx armPort_POW = null;
    public DcMotorEx armStar = null;

    public Servo servoHand = null;
    public Servo servoTurret = null;
    public Servo servoExtend = null;
    public DcMotor extendEncoder = null;
    //public Servo servoPoleToucherPort = null;
    //public Servo servoPoleToucherStar = null;

    public ColorSensor colorSensorStar = null;

    //public TouchSensor touchSensorPort = null;
    //public TouchSensor touchSensorStar = null;

    public DistanceSensor distSensorHand = null;
    public DistanceSensor distSensorPort = null;
    public DistanceSensor distSensorStar = null;

    public RevBlinkinLedDriver lights = null;

    private ElapsedTime runtime = new ElapsedTime();

    public final double handOpen = .75;//static?
    public final double handClosed = 0;//static?

    public final double turretForward = .6;//static?
    public final double turretPort = 1;//static?
    public final double turretStar = 0.05;//static?

    static final double COUNTS_PER_MOTOR_REV_BE = 8192;
    static final double DRIVE_GEAR_REDUCTION_BE = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES_BE = 3.5;     // For figuring circumference
    public static final double COUNTS_PER_INCH_BE = (COUNTS_PER_MOTOR_REV_BE * DRIVE_GEAR_REDUCTION_BE) /
            (WHEEL_DIAMETER_INCHES_BE * 3.1415);

    public final double maxVelocity = 3250;//this is an inversion of the scaling that's happening in the tuning class


    HardwareMap hwMap = null;

    public Tamaru3Hardware(){
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

        BOW = hwMap.get(DcMotorEx.class, "BOW");
        SOW = hwMap.get(DcMotorEx.class, "SOW");

        armPort_POW = hwMap.get(DcMotorEx.class, "armPort_POW");
        armStar = hwMap.get(DcMotorEx.class, "armStar");

        servoHand = hwMap.get(Servo.class, "servoHand");
        servoTurret = hwMap.get(Servo.class, "servoTurret");
        servoExtend = hwMap.get(Servo.class, "servoExtend");

        //servoPoleToucherPort = hwMap.get(Servo.class, "servoPoleToucherPort");
        //servoPoleToucherStar = hwMap.get(Servo.class, "servoPoleToucherStar");

        colorSensorStar = hwMap.get(ColorSensor.class, "colorSensorStar");

        distSensorHand = hwMap.get(DistanceSensor.class, "distSensorHand");
        distSensorPort = hwMap.get(DistanceSensor.class, "distSensorPort");
        distSensorStar = hwMap.get(DistanceSensor.class, "distSensorStar");

        //touchSensorPort = hwMap.get(TouchSensor.class, "touchSensorPort");
        //touchSensorStar = hwMap.get(TouchSensor.class, "touchSensorStar");


        lights = hwMap.get(RevBlinkinLedDriver.class, "lights");

        //flipped drive directions 2/25 11:29
        fpd.setDirection(DcMotorSimple.Direction.REVERSE);
        fsd.setDirection(DcMotorSimple.Direction.REVERSE);
        bpd.setDirection(DcMotorSimple.Direction.REVERSE);
        bsd.setDirection(DcMotorSimple.Direction.FORWARD);

        SOW.setDirection(DcMotorSimple.Direction.FORWARD);
        BOW.setDirection(DcMotorSimple.Direction.FORWARD);
        armPort_POW.setDirection(DcMotorSimple.Direction.FORWARD);

        armStar.setDirection(DcMotorSimple.Direction.FORWARD);

        fpd.setPower(0);
        fsd.setPower(0);
        bpd.setPower(0);
        bsd.setPower(0);

        armPort_POW.setPower(0);
        armStar.setPower(0);

        fpd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fsd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bpd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bsd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armPort_POW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armStar.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fpd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fsd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bpd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bsd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armPort_POW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armStar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
}
