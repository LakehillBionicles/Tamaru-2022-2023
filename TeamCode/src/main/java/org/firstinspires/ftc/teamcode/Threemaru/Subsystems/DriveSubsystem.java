package org.firstinspires.ftc.teamcode.Threemaru.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveSubsystem extends SubsystemBase {
    private final DcMotorEx fpd, bpd, fsd, bsd;


    public DriveSubsystem(DcMotorEx fpd, DcMotorEx bpd, DcMotorEx fsd, DcMotorEx bsd) {
        this.fpd = fpd; this.bpd = bpd; this.fsd = fsd; this.bsd = bsd;

        fpd.setDirection(DcMotorEx.Direction.FORWARD);
        bpd.setDirection(DcMotorEx.Direction.FORWARD);
        fsd.setDirection(DcMotorEx.Direction.FORWARD);
        bsd.setDirection(DcMotorEx.Direction.REVERSE);

        fpd.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        bpd.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        fsd.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        bsd.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        fpd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bpd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fsd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bsd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setDrivePower(double drivePower, double strafePower, double rotPower){
        fpd.setPower(drivePower - strafePower + rotPower);
        bpd.setPower(drivePower + strafePower + rotPower);
        fsd.setPower(drivePower + strafePower - rotPower);
        bsd.setPower(drivePower - strafePower - rotPower);
    }

    public void resetDrive(){
        fpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fpd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bpd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fsd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bsd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
