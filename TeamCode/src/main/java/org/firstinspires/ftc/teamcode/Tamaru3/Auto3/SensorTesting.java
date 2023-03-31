package org.firstinspires.ftc.teamcode.Tamaru3.Auto3;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class SensorTesting extends AutoBase{

    @Override
    public void runOpMode() {
        super.runOpMode();
        robot.init(hardwareMap);

        waitForStart();

        while(opModeIsActive()){
            robot.servoTurret.setPosition(robot.turretPort);

            double pos = 1;

            if(gamepad2.dpad_up) {
                pos = .95;
            } else if(gamepad2.dpad_right) {
                pos = .9;
            } else if(gamepad2.dpad_down) {
                pos = .85;
            } else if(gamepad2.dpad_left) {
                pos = .8;
            } else if(gamepad2.y) {
                pos = .7;
            } else if(gamepad2.b) {
                pos = .65;
            } else if(gamepad2.a) {
                pos = .6;
            } else if(gamepad2.x) {
                pos = .55;
            }

            if(gamepad2.left_bumper){
                robot.servoHand.setPosition(robot.handClosed);
            } else if(gamepad2.right_bumper){
                robot.servoHand.setPosition(robot.handOpen);
            }

            robot.servoExtend.setPosition(pos);
            //double distPort = (robot.distSensorPort.getDistance(DistanceUnit.CM)+robot.distSensorPort2.getDistance(DistanceUnit.CM))/2;
            //double distStar = robot.distSensorStar.getDistance(DistanceUnit.CM);

            //robot.servoExtend.setPosition(1.33 + -0.188*distPort + 0.0104*distPort*distPort);
            //robot.servoExtend.setPosition(1.45 + -0.145x + 4.76E-03x^2);

            //robot.servoExtend.setPosition((1.33+1.45)/2 + -0.188*distPort + -0.145*distStar);

            telemetry.addData("distStar1", robot.distSensorStar.getDistance(DistanceUnit.CM));
            telemetry.addData("distPort1", robot.distSensorPort.getDistance(DistanceUnit.CM));
            telemetry.addData("distStar2", robot.distSensorStar2.getDistance(DistanceUnit.CM));
            telemetry.addData("distPort2", robot.distSensorPort2.getDistance(DistanceUnit.CM));
            telemetry.update();
        }

        //driveUntilDist();
        //lineUpWithConeStack();

        telemetry.addData("out", "out");
        telemetry.addData("color", senseColors());
        telemetry.addData("red", robot.colorSensorBottom.red());
        telemetry.addData("blue", robot.colorSensorBottom.blue());
        telemetry.addData("green", robot.colorSensorBottom.green());
        telemetry.update();

        sleep(100000);
    }

    public void driveUntilDist(){
        while(robot.distSensorStar.getDistance(DistanceUnit.CM)>10){
            /*robot.fpd.setPower(-.25);
            robot.bpd.setPower(-.25);
            robot.fsd.setPower(-.25);
            robot.bsd.setPower(-.25);*/

            telemetry.addData("dist", robot.distSensorPort.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
        robot.fpd.setPower(0);
        robot.bpd.setPower(0);
        robot.fsd.setPower(0);
        robot.bsd.setPower(0);
    }

    public void lineUpWithConeStack(){
        while(!senseColors().equals("red")){
            robot.fpd.setPower(.5);
            robot.bpd.setPower(-.5);
            robot.fsd.setPower(-.5);
            robot.bsd.setPower(.5);
        }
        robot.fpd.setPower(0);
        robot.bpd.setPower(0);
        robot.fsd.setPower(0);
        robot.bsd.setPower(0);
    }

    public String senseColors() {
        String color = "blank";
        double redMax = robot.colorSensorBottom.red();
        int blueMax = robot.colorSensorBottom.blue();
        int greenMax = robot.colorSensorBottom.green();

        while (opModeIsActive() && color.equals("blank")) {
            if ((redMax > blueMax) && (redMax > greenMax)) {
                telemetry.addData("i see red", " ");
                telemetry.update();
                color = "red";
            } else if ((blueMax > redMax) && (blueMax > greenMax)) {
                telemetry.addData("i see blue", " ");
                telemetry.update();
                color = "blue";
            } else if ((greenMax > redMax) && (greenMax > blueMax)) {
                telemetry.addData("i see green", " ");
                telemetry.update();
                color = "green";
            } else {
                telemetry.addData("i see nothing", " ");
                telemetry.update();
                color = "no go";
            }

        }
        return color;
    }
}
