package org.firstinspires.ftc.teamcode.CommandBasedTesting.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
/**
 * Arm Subsystem
 * 4 motors
 * enum Height has heights for the ground, heights for the cone stacks, and heights for the pole
 * Command setArmPower() sets the arm motors to any power
 * Command setArmToHeight() sets the arm to a height (Height enum) using a PIDController called armPID
 */
@Config
public class LinearArmSubsystem extends SubsystemBase {
    private final DcMotorEx arm1, arm2, arm3, arm4;

    public static double kP = 0, kI = 0, kD = 0;
    public static double tolerance = 10;
    public static int currentHeight = 0;

    private PIDController armPID = new PIDController(kP, kI, kD);

    public enum Height {
        GROUND(0),
        TWO_CONES(300), THREE_CONES(400), FOUR_CONES(500), FIVE_CONES(600),
        LOW_POLE(1400), MID_POLE(2000), HIGH_POLE(2500);

        public final int height;

        Height(int height) {
            this.height = height;
        }

        public int getHeight() {
            return height;
        }
    }

    public LinearArmSubsystem(DcMotorEx motor1, DcMotorEx motor2, DcMotorEx motor3, DcMotorEx motor4){
        this.arm1 = motor1; this.arm2 = motor2; this.arm3 = motor3; this.arm4 = motor4;
        armPID.setTolerance(tolerance);
        armPID.setSetPoint(0);

        arm1.setDirection(DcMotorEx.Direction.FORWARD);
        arm2.setDirection(DcMotorEx.Direction.FORWARD);
        arm3.setDirection(DcMotorEx.Direction.FORWARD);
        arm4.setDirection(DcMotorEx.Direction.FORWARD);

        arm1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        arm2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        arm3.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        arm4.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        arm1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        arm2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        arm3.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        arm4.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    private void setHeight(Height height) {
        currentHeight = height.getHeight();
        armPID.setSetPoint(height.getHeight());
    }

    public int getArmPos() {
        return arm1.getCurrentPosition();
    }

    public boolean atTarget() {
        return getArmPos() < currentHeight + tolerance && getArmPos() < currentHeight - tolerance;
    }

    public Command setArmPower(double power) {
        return new RunCommand(()-> {
            arm1.setPower(power); arm2.setPower(power); arm3.setPower(power); arm4.setPower(power);
        }, this);
    }

    public Command setArmToHeight(Height height) {
        return new RunCommand(() -> {
            setHeight(height);
            while(!atTarget()){
                double power = armPID.calculate(getArmPos());
                arm1.setPower(power); arm2.setPower(power); arm3.setPower(power); arm4.setPower(power);
            }
         }, this);
    }

}
