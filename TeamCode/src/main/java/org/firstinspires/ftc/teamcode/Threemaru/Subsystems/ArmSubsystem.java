package org.firstinspires.ftc.teamcode.Threemaru.Subsystems;

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
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class ArmSubsystem extends SubsystemBase {
    DcMotorEx armPort = null, armStar = null;

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

    public ArmSubsystem(HardwareMap HardwareMap){
        armPort = HardwareMap.get(DcMotorEx.class, "armPort");
        armStar = HardwareMap.get(DcMotorEx.class, "armStar");

        armPort.setDirection(DcMotorEx.Direction.FORWARD);
        armStar.setDirection(DcMotorEx.Direction.FORWARD);

        armPort.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        armStar.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        armPort.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        armStar.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void setArmPower(double power) {
        armPort.setPower(power);
        armStar.setPower(power);
    }

    public void armToPosition(Height targetHeight, double power){
        setArmTarget(targetHeight.getHeight());
        setArmMode(DcMotor.RunMode.RUN_TO_POSITION);
        setArmPower(power);
    }

    public void setArmMode(DcMotorEx.RunMode runMode){
        armPort.setMode(runMode);
        armStar.setMode(runMode);
    }

    public void setArmTarget(int targetPosition){
        armPort.setTargetPosition(targetPosition);
        armStar.setTargetPosition(targetPosition);
    }

    public void resetArm(){
        armPort.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armStar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armPort.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armStar.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

}
