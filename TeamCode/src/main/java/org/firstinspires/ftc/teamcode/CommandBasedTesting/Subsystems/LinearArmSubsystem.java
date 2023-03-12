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
 * Command setArmToHeight() sets the arm to a height (Height enum) using RUN_TO_POSITION
 */
@Config
public class LinearArmSubsystem extends SubsystemBase {
    private final DcMotorEx armPortI, armPortO, armStarI, armStarO;

    public static double kP = 0, kI = 0, kD = 0;
    public static double tolerance = 10;
    public static int currentHeight = 0;

    private PIDController armPID = new PIDController(kP, kI, kD);

    public enum Height {
        GROUND(0),
        TWO_CONES(300), THREE_CONES(400), FOUR_CONES(500), FIVE_CONES(600),
        LOW_POLE(1400), MID_POLE(2000), HIGH_POLE(2900);

        public final int height;

        Height(int height) {
            this.height = height;
        }

        public int getHeight() {
            return height;
        }
    }

    public LinearArmSubsystem(DcMotorEx motor1, DcMotorEx motor2, DcMotorEx motor3, DcMotorEx motor4){
        this.armPortI = motor1; this.armPortO = motor2; this.armStarI = motor3; this.armStarO = motor4;
        armPID.setTolerance(tolerance);
        armPID.setSetPoint(0);

        armPortI.setDirection(DcMotorEx.Direction.REVERSE);
        armPortO.setDirection(DcMotorEx.Direction.FORWARD);
        armStarI.setDirection(DcMotorEx.Direction.REVERSE);
        armStarO.setDirection(DcMotorEx.Direction.FORWARD);

        armPortI.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        armPortO.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        armStarI.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        armStarO.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        armPortI.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        armPortO.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        armStarI.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        armStarO.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public int getArmPos() {
        return ((armPortI.getCurrentPosition()+armPortO.getCurrentPosition()+armStarI.getCurrentPosition()+armStarO.getCurrentPosition())/4);
    }

    public boolean atTarget() {
        return getArmPos() < currentHeight + tolerance && getArmPos() < currentHeight - tolerance;
    }

    public Command setArmPower(double power) {
        return new RunCommand(()-> {
            armPortI.setPower(power); armPortO.setPower(power); armStarI.setPower(power); armStarO.setPower(power);
        }, this);
    }

    public void setArmMode(DcMotorEx.RunMode runMode){
        armPortI.setMode(runMode);
        armPortO.setMode(runMode);
        armStarI.setMode(runMode);
        armStarO.setMode(runMode);
    }

    public void setArmTarget(int targetPosition){
        armPortI.setTargetPosition(targetPosition);
        armPortO.setTargetPosition(targetPosition);
        armStarI.setTargetPosition(targetPosition);
        armStarO.setTargetPosition(targetPosition);
    }

    public Command setArmToHeight(Height height) {
        return new InstantCommand(() -> {
            setArmTarget(height.getHeight());
            setArmMode(DcMotor.RunMode.RUN_TO_POSITION);
            setArmPower(1);
         }, this);
    }

    public void resetArm(){
        armPortI.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armPortO.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armStarI.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armStarO.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armPortI.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armPortO.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armStarI.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armStarO.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

}
