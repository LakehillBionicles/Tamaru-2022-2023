package org.firstinspires.ftc.teamcode.CommandBasedTesting.Subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.function.DoubleSupplier;

/**
 * DrivetrainSubsystem
 * four motors
 * Command drive() sets the motor powers based on drivePower, strafePower, and rotPower
 */
public class DrivetrainSubsystem extends SubsystemBase {
    private final DcMotorEx fpd, bpd, fsd, bsd;

    public static final double TICKS_PER_REV = 28;
    public static double WHEEL_RADIUS = 1.8898;
    public static double GEAR_RATIO = 1/10.4329;

    public DrivetrainSubsystem(DcMotorEx fpd, DcMotorEx bpd, DcMotorEx fsd, DcMotorEx bsd) {
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

    public Command drive(double drivePower, double strafePower, double rotPower) {return new RunCommand(() -> {
            fpd.setPower(drivePower - strafePower + rotPower);
            bpd.setPower(drivePower + strafePower + rotPower);
            fsd.setPower(drivePower + strafePower - rotPower);
            bsd.setPower(drivePower - strafePower - rotPower);
        });
    }

    public void setDriveVelocity(double driveVelocity, double strafeVelocity, double rotVelocity){
        fpd.setVelocity(driveVelocity - strafeVelocity + rotVelocity);
        bpd.setVelocity(driveVelocity + strafeVelocity + rotVelocity);
        fsd.setVelocity(driveVelocity + strafeVelocity - rotVelocity);
        bsd.setVelocity(driveVelocity - strafeVelocity - rotVelocity);
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public double avgEncoderTicks() {
        return (fpd.getCurrentPosition()+bpd.getCurrentPosition()+fsd.getCurrentPosition()+bsd.getCurrentPosition())/4.0;
    }

    public double getYDistance() {
        return encoderTicksToInches(avgEncoderTicks());
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
