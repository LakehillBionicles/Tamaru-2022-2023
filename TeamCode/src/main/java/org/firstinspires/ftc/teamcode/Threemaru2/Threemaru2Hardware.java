package org.firstinspires.ftc.teamcode.Threemaru2;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Threemaru2Hardware extends LinearOpMode {
    public DcMotorEx fpd = null, fsd = null, bpd = null, bsd = null, POW = null;

    public DcMotorEx armPort = null, armStar = null;

    public DcMotorEx motorTurret = null;

    public Servo servoHand1 = null, servoHand2 = null;;
    public Servo servoExtend = null;

    public DistanceSensor distSensorPort = null, distSensorStar = null, distSensorHand = null;

    public static final double COUNTS_PER_MOTOR_REV = 560, DRIVE_GEAR_REDUCTION = 1.0, WHEEL_DIAMETER_INCHES = 1.8898 * 2;
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    public ElapsedTime runtime = new ElapsedTime();

    HardwareMap hwMap = null;

    public Threemaru2Hardware() {}

    @Override
    public void runOpMode() {
    }

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        fpd = hwMap.get(DcMotorEx.class, "fpd");
        fsd = hwMap.get(DcMotorEx.class, "fsd");
        bpd = hwMap.get(DcMotorEx.class, "bpd");
        bsd = hwMap.get(DcMotorEx.class, "bsd");
        //POW = hwMap.get(DcMotorEx.class, "POW");

        armPort = hwMap.get(DcMotorEx.class, "armPort");
        armStar = hwMap.get(DcMotorEx.class, "armStar");

        motorTurret = hwMap.get(DcMotorEx.class, "motorTurret");

        servoHand1 = hwMap.get(Servo.class, "servoHand1");
        servoHand2 = hwMap.get(Servo.class, "servoHand2");
        servoExtend = hwMap.get(Servo.class, "servoExtend");

        distSensorPort = hwMap.get(DistanceSensor.class, "distSensorPort");
        distSensorStar = hwMap.get(DistanceSensor.class, "distSensorStar");
        distSensorHand = hwMap.get(DistanceSensor.class, "distSensorHand");

        fpd.setDirection(DcMotorSimple.Direction.REVERSE);//F
        fsd.setDirection(DcMotorSimple.Direction.REVERSE);//R
        bpd.setDirection(DcMotorSimple.Direction.FORWARD);//R
        bsd.setDirection(DcMotorSimple.Direction.REVERSE);//F
        //POW.setDirection(DcMotorSimple.Direction.REVERSE);

        armPort.setDirection(DcMotorSimple.Direction.REVERSE);
        armStar.setDirection(DcMotorSimple.Direction.FORWARD);

        motorTurret.setDirection(DcMotorSimple.Direction.FORWARD);

        fpd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fsd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bpd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bsd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //POW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armPort.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armStar.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorTurret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fpd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fsd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bpd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bsd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //POW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armPort.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armStar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorTurret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
