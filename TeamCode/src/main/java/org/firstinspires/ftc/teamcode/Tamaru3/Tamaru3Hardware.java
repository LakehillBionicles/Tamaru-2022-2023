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

    public DcMotorEx armPortI = null;
    public DcMotorEx armPortO = null;
    public DcMotorEx armStarI = null;
    public DcMotorEx armStarO = null;

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

    public final double handOpen = .8, handClosed = 0;

    public final double turretForward = .285, turretPort = 0, turretStar = .6;

    public final double extensionPort = .5, extensionStar = .5;

    static final double COUNTS_PER_MOTOR_REV_BE = 8192;
    static final double DRIVE_GEAR_REDUCTION_BE = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES_BE = 3.5;     // For figuring circumference
    public static final double COUNTS_PER_INCH_BE = (COUNTS_PER_MOTOR_REV_BE * DRIVE_GEAR_REDUCTION_BE) /
            (WHEEL_DIAMETER_INCHES_BE * 3.1415);

    public final double maxVelocity = 3250;


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

        armPortI = hwMap.get(DcMotorEx.class, "armPortI");
        armPortO = hwMap.get(DcMotorEx.class, "armPortO");
        armStarI = hwMap.get(DcMotorEx.class, "armStarI");
        armStarO = hwMap.get(DcMotorEx.class, "armStarO");

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
        fpd.setDirection(DcMotorSimple.Direction.REVERSE);//BOW
        fsd.setDirection(DcMotorSimple.Direction.REVERSE);
        bpd.setDirection(DcMotorSimple.Direction.FORWARD);//POW
        bsd.setDirection(DcMotorSimple.Direction.FORWARD);//SOW

        armPortI.setDirection(DcMotorSimple.Direction.REVERSE);//formerly BOW
        armPortO.setDirection(DcMotorSimple.Direction.FORWARD);//formerly POW
        armStarI.setDirection(DcMotorSimple.Direction.REVERSE);
        armStarO.setDirection(DcMotorSimple.Direction.FORWARD);//formerly SOW

        fpd.setPower(0);
        fsd.setPower(0);
        bpd.setPower(0);
        bsd.setPower(0);

        armPortI.setPower(0);
        armPortO.setPower(0);
        armStarI.setPower(0);
        armStarO.setPower(0);

        fpd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fsd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bpd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bsd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armPortI.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armPortO.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armStarI.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armStarO.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fpd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fsd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bpd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bsd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armPortI.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armPortO.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armStarI.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armStarO.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
