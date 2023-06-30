package org.firstinspires.ftc.teamcode.NewRoadRunnerTest.Testing;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class TestHwMap extends LinearOpMode {
    public DcMotorEx fpd = null;
    public DcMotorEx fsd = null;
    public DcMotorEx bpd = null;
    public DcMotorEx bsd = null;

    public DcMotor POW = null;
    public DcMotor SOW = null;
    public DcMotor BOWF = null;
    public DcMotor BOWB = null;


    public ElapsedTime runtime = new ElapsedTime();

    HardwareMap hwMap = null;

    public TestHwMap() {
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

        POW = hwMap.get(DcMotorEx.class, "POW");
        SOW = hwMap.get(DcMotorEx.class, "SOW");
        BOWF = hwMap.get(DcMotorEx.class, "BOWF");
        BOWB = hwMap.get(DcMotorEx.class, "BOWB");

        fpd.setDirection(DcMotorSimple.Direction.REVERSE);
        fsd.setDirection(DcMotorSimple.Direction.REVERSE);
        bpd.setDirection(DcMotorSimple.Direction.FORWARD);
        bsd.setDirection(DcMotorSimple.Direction.FORWARD);

        POW.setDirection(DcMotorSimple.Direction.REVERSE);
        SOW.setDirection(DcMotorSimple.Direction.REVERSE);
        BOWF.setDirection(DcMotorSimple.Direction.REVERSE);
        BOWB.setDirection(DcMotorSimple.Direction.REVERSE);//not confirmed

        fpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fsd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bsd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        POW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SOW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BOWF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BOWB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fpd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fsd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bpd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bsd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        POW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SOW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BOWF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BOWB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
