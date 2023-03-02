package org.firstinspires.ftc.teamcode.Tamaru2.BaseClasses;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Objects;

public abstract class executionClass extends calculationClass {
    public Tamaru2Hardware robot = new Tamaru2Hardware();

    public void execution(double fpdPOWPower, double bpdBOWPower, double fsdSOWPower, double bsdPower, double driveDenom, double extendPower,
                          double handPosition, double turretPosition, int armTargetPosition, double armPower, boolean armToHeight, String mode){
        robot.init(hardwareMap);

        if(Objects.equals(mode, "tele")) {
            fieldCentricCalculations();
            robot.fpd.setPower(fpdPOWPower/driveDenom);
            robot.bpd.setPower(bpdBOWPower/driveDenom);
            robot.fsd.setPower(fsdSOWPower/driveDenom);
            robot.bsd.setPower(bsdPower/driveDenom);
        } else if(Objects.equals(mode, "auto")) {
            robot.fpd.setPower(fpdPOWOdoPower);
            robot.bpd.setPower(bpdBOWOdoPower);
            robot.fsd.setPower(fsdSOWOdoPower);
            robot.bsd.setPower(bsdOdoPower);
        }

        //robot.servoExtend.setPower(extendPower);

        robot.servoHand.setPosition(handPosition);
        robot.servoTurret.setPosition(turretPosition);

        if (armToHeight){
            robot.armPort.setTargetPosition(armTargetPosition);
            robot.armPort.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.armPort.setPower(armPower);
            robot.armStar.setPower(armPower);
        } else if (!armToHeight) {
            robot.armPort.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.armStar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.armPort.setPower(armPower);
            robot.armStar.setPower(armPower);
        }

    }
}