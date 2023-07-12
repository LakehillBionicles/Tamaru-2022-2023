package org.firstinspires.ftc.teamcode.Threemaru2;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Threemaru2Hardware extends LinearOpMode {
    public DcMotorEx fpd = null, fsd = null, bpd = null, bsd = null;

    public DcMotor POW = null, SOW = null, BOWF = null, BOWB = null;

    public ElapsedTime runtime = new ElapsedTime();

    HardwareMap hwMap = null;

    public Threemaru2Hardware() {}

    @Override
    public void runOpMode() {}

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
        BOWB.setDirection(DcMotorSimple.Direction.REVERSE);

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
