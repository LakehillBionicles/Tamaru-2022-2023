package org.firstinspires.ftc.teamcode.Tamaru1;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TemaruHardware extends LinearOpMode {

    public DcMotor fpd = null;
    public DcMotor fsd = null;
    public DcMotor bpd = null;
    public DcMotor bsd = null;

    public DcMotor BOW = null;
    public DcMotor arm2 = null;

    public DcMotor POW = null; //these are the odometery wheels
    public DcMotor SOW = null;

    //public DcMotor hand = null;
    public Servo servoFinger = null;
    public CRServo servoArm = null;

    public ColorSensor colorSensorPort = null;
    public ColorSensor colorSensorStar = null;

    public ColorSensor colorSensorHand = null;

    public DistanceSensor distSensorHand = null;

    public DistanceSensor distSensorArm = null;
    public DistanceSensor distSensorHighArm = null;
    public DistanceSensor distSensorLowerArm = null;
    public DistanceSensor distSensorMiddleArm = null;

    public TouchSensor touchSensorArm = null;

    public TouchSensor touchSensorPort = null;
    public TouchSensor touchSensorStar = null;

    //public TouchSensor magnet = null;

    public RevBlinkinLedDriver lights = null;

    public static final double armSpeed = 1.0;
    public static final double openHandPos = 1.0;
    public static final double closeHandPos = 0.0;


    ////////////////////////////////////////////////////  ELBOW VARIABLES ///////////////////////////////////////////////
   /* public double elbowDenominator;
    public double elbowPosition;
    public double elbowError;
    public double lastElbowError;
    public double elbowTime = 0;
    public double newElbowTarget;
    public double elbowDeriv;
    public double elbowIntegralSum = 0;
    public double elbowIntegralSumLimit = 0.25;

    public double elbowKp = 1.0;
    public double elbowKd = 1.0;
    public double elbowKi = 1.0;

    public double elbowPower;
*/



    private ElapsedTime runtime = new ElapsedTime();


    static final double COUNTS_PER_MOTOR_REV_BE = 8192;
    static final double DRIVE_GEAR_REDUCTION_BE = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES_BE = 3.5;     // For figuring circumference
    public static final double COUNTS_PER_INCH_BE = (COUNTS_PER_MOTOR_REV_BE * DRIVE_GEAR_REDUCTION_BE) /
            (WHEEL_DIAMETER_INCHES_BE * 3.1415);


    HardwareMap hwMap = null;
    private ElapsedTime time = new ElapsedTime();

    /* Constructor */
    public TemaruHardware() {
    }

    @Override

    public void runOpMode() {
    }

    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;


        fpd = hwMap.get(DcMotor.class, "fpd");
        fsd = hwMap.get(DcMotor.class, "fsd");
        bpd = hwMap.get(DcMotor.class, "bpd");
        bsd = hwMap.get(DcMotor.class, "bsd");

        BOW = hwMap.get(DcMotor.class, "BOW");
        arm2 = hwMap.get(DcMotor.class, "arm2");

        POW = hwMap.get(DcMotor.class, "POW");
        SOW = hwMap.get(DcMotor.class, "SOW");

        //hand = hwMap.get(DcMotor.class, "hand");
        servoFinger = hwMap.get(Servo.class, "servoFinger");
        servoArm = hwMap.get(CRServo.class, "servoArm");

        colorSensorPort = hwMap.get(ColorSensor.class, "colorSensorPort");
        colorSensorStar = hwMap.get(ColorSensor.class, "colorSensorStar");

        colorSensorHand = hwMap.get(ColorSensor.class, "colorSensorHand");


        distSensorHand = hwMap.get(DistanceSensor.class, "distSensorHand");

        distSensorArm = hwMap.get(DistanceSensor.class, "distSensorArm");
        distSensorHighArm = hwMap.get(DistanceSensor.class, "distSensorHighArm");
        distSensorLowerArm = hwMap.get(DistanceSensor.class, "distSensorLowerArm");
        distSensorMiddleArm = hwMap.get(DistanceSensor.class, "distSensorMiddleArm");

        touchSensorArm = hwMap.get(TouchSensor.class, "touchSensorArm");

        touchSensorPort = hwMap.get(TouchSensor.class, "touchSensorPort");
        touchSensorStar = hwMap.get(TouchSensor.class, "touchSensorStar");



        //hand = hwMap.get(DcMotor.class, "hand");

        lights = hwMap.get(RevBlinkinLedDriver.class, "lights");



        //set directions

        fsd.setDirection(DcMotorSimple.Direction.FORWARD); //r
        bsd.setDirection(DcMotorSimple.Direction.FORWARD);//f
        fpd.setDirection(DcMotorSimple.Direction.FORWARD);//f
        bpd.setDirection(DcMotorSimple.Direction.FORWARD); //r





        //DO NOT CHANGE ARM DIRECTIONS EVER EVER EVER EVER EVER EVER EVER EVER EVER OR ELSE

        POW.setDirection(DcMotorSimple.Direction.REVERSE); //changed from FORWARD to REVERSE (1/23 1:41)
        BOW.setDirection(DcMotorSimple.Direction.FORWARD); //flipped BOW direction

        SOW.setDirection(DcMotorSimple.Direction.REVERSE); //flipped SOW direction
        arm2.setDirection(DcMotorSimple.Direction.FORWARD);




        //set power to 0
        fpd.setPower(0);
        fsd.setPower(0);
        bpd.setPower(0);
        bsd.setPower(0);

        POW.setPower(0);
        SOW.setPower(0);
        BOW.setPower(0);
        arm2.setPower(0);

        //hand.setPower(0);


        //encoder run
        fpd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fsd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bpd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bsd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        BOW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        POW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SOW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        arm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //hand.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        //brake behavior
        fpd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fsd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bpd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bsd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        POW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SOW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BOW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); //used to be brake

        //hand.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
}
