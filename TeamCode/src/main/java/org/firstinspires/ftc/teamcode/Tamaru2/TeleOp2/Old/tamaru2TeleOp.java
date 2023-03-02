package org.firstinspires.ftc.teamcode.Tamaru2.TeleOp2.Old;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Tamaru2.BaseClasses.Tamaru2Hardware;
import org.firstinspires.ftc.teamcode.Tamaru2.BaseClasses.calculationClass;
import org.firstinspires.ftc.teamcode.Tamaru2.BaseClasses.executionClass;


@TeleOp
//@Disabled

    /*
    DRIVE
    EXTEND
    HAND
    TURRET
    ARM
    EXECUTION
     */

public class tamaru2TeleOp extends executionClass {

    Tamaru2Hardware robot = new Tamaru2Hardware();

    //changed all from private to public 2/5 3:48
    public double fpdPOWPower = 0;
    public double bpdBOWPower = 0;
    public double fsdSOWPower = 0;
    public double bsdPower = 0;
    public double extendPower = 0;
    public double handPosition = handClosed;
    public double turretPosition = turretForward;
    public int armTargetPosition = 0;
    public double armPower = 0;
    public boolean armToHeight = true;
    public boolean fieldCentric = true;
    public double drivePower;
    public double strafePower;
    public double rotatePower;
    public double driveDenom = 1;
    public String mode = "tele";


    public ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        startUp();

        waitForStart();

        while (opModeIsActive()) {

            ///////////////////////////////////////DRIVE////////////////////////////////////////////////////////
            drivePower = gamepad1.left_stick_y;
            strafePower = -gamepad1.left_stick_x;
            rotatePower = -gamepad1.right_stick_x;

            if(gamepad1.dpad_up){
                fieldCentricUp = true;
                fieldCentricDown = false;
                fieldCentricLeft = false;
                fieldCentricRight = false;
            } else if(gamepad1.dpad_down){
                fieldCentricUp = false;
                fieldCentricDown = true;
                fieldCentricLeft = false;
                fieldCentricRight = false;
            } else if(gamepad1.dpad_left){
                fieldCentricUp = false;
                fieldCentricDown = false;
                fieldCentricLeft = true;
                fieldCentricRight = false;
            } else if(gamepad1.dpad_right){
                fieldCentricUp = false;
                fieldCentricDown = false;
                fieldCentricLeft = false;
                fieldCentricRight = true;
            }

            if(gamepad1.left_trigger>0){
                driveDenom = .5;
            } else {
                driveDenom = 1;
            }

            if(gamepad1.a){
                fieldCentric = false;
            } else if(gamepad1.b){
                fieldCentric = true;
            }

            if(fieldCentric) {
                fpdPOWPower = fpdPOWFCPower;
                bpdBOWPower = bpdBOWFCPower;
                fsdSOWPower = fpdPOWFCPower;
                bsdPower = fpdPOWFCPower;
            } else {
                fpdPOWPower = drivePower + strafePower + rotatePower;
                bpdBOWPower = drivePower - strafePower + rotatePower;
                fsdSOWPower = drivePower - strafePower - rotatePower;
                bsdPower = drivePower + strafePower - rotatePower;
            }

            ///////////////////////////////////////EXTEND////////////////////////////////////////////////////////
            if(gamepad2.right_stick_y < 0){
                extendPower = 1;
            } else if(gamepad2.right_stick_y > 0){
                extendPower = -1;
            } else{
                extendPower = 0;
            }

            ///////////////////////////////////////HAND////////////////////////////////////////////////////////
            if (gamepad2.left_bumper || gamepad1.left_bumper) {
                handPosition = handClosed;
            } else if (gamepad2.right_bumper || gamepad1.right_bumper) {
                handPosition = handOpen;
            }

            ///////////////////////////////////////TURRET////////////////////////////////////////////////////////
            if(gamepad2.dpad_up){
                turretPosition = turretForward;
            } else if (gamepad2.dpad_right){
                turretPosition = turretStar;
            } else if (gamepad2.dpad_left){
                turretPosition = turretPort;
            }

            ///////////////////////////////////////ARM////////////////////////////////////////////////////////
            if(gamepad2.left_trigger>0){
                armToHeight = false;
            } else if(gamepad2.right_trigger>0){
                armToHeight = true;
            }

            if(gamepad2.a && armToHeight){
                armTargetPosition = downArmTarget;
            } else if(gamepad2.x && armToHeight){
                armTargetPosition = lowArmTarget;
            } else if(gamepad2.y && armToHeight){
                armTargetPosition = midArmTarget;
            } else if(gamepad2.b && armToHeight){
                armTargetPosition = highArmTarget;
            }

            if(!armToHeight){
                if(gamepad2.left_stick_y<0){
                    armPower = -gamepad2.left_stick_y;
                } else if(gamepad2.left_stick_y>0){
                    armPower = -gamepad2.left_stick_y/2;
                }
            }

            ///////////////////////////////////////EXECUTION////////////////////////////////////////////////////////
            execution(fpdPOWPower, bpdBOWPower, fsdSOWPower, bsdPower, driveDenom, extendPower, handPosition, turretPosition, armTargetPosition, armPower, armToHeight, mode);
                /*if(mode=="tele") {
                    fieldCentricCalculations();

                    robot.fpdPOW.setPower(fpdPOWPower/driveDenom);
                    robot.bpdBOW.setPower(bpdBOWPower/driveDenom);
                    robot.fsdSOW.setPower(fsdSOWPower/driveDenom);
                    robot.bsd.setPower(bsdPower/driveDenom);
                } else if(mode=="auto") {
                    robot.fpdPOW.setPower(fpdPOWOdoPower);
                    robot.bpdBOW.setPower(bpdBOWOdoPower);
                    robot.fsdSOW.setPower(fsdSOWOdoPower);
                    robot.bsd.setPower(bsdOdoPower);
                }

                robot.servoExtend.setPower(extendPower);

                robot.servoHand.setPosition(handPosition);
                robot.servoTurret.setPosition(turretPosition);

                if (armToHeight == true){
                    robot.armPort.setTargetPosition(armTargetPosition);
                    robot.armPort.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.armPort.setPower(armPower);
                    robot.armStar.setPower(armPower);
                } else if (armToHeight == false) {
                    robot.armPort.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.armStar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.armPort.setPower(armPower);
                    robot.armStar.setPower(armPower);
                }

                 */

        }
    }
}
