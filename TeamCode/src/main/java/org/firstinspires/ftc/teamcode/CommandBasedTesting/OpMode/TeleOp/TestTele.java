package org.firstinspires.ftc.teamcode.CommandBasedTesting.OpMode.TeleOp;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.*;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandBasedTesting.OpMode.TestBaseOpMode;

/** TestTele
 * attempting to use TestBaseOpMode for TeleOp
 * controls drive, arm, hand, and turret
 * baseControl is gamepad1, armControl is gamepad2
 */
@TeleOp
@Config
public class TestTele extends TestBaseOpMode {

    @Override
    public void initialize() { super.initialize(); }

    @Override
    public void run() {
        super.run();
        /*hand*/
        baseControlButton(LEFT_BUMPER).whenPressed(tamaruHand.grab());
        baseControlButton(RIGHT_BUMPER).whenPressed(tamaruHand.release());
        armControlButton(LEFT_BUMPER).whenPressed(tamaruHand.grab());
        armControlButton(RIGHT_BUMPER).whenPressed(tamaruHand.release());
        /*turret*/
        armControlButton(DPAD_UP).whenPressed(tamaruTurret.setTurretForward());
        armControlButton(DPAD_LEFT).whenPressed(tamaruTurret.setTurretPort());
        armControlButton(DPAD_RIGHT).whenPressed(tamaruTurret.setTurretStar());
        /*drive*/
        tamaruDrivetrain.drive(-baseControl.getLeftY(), baseControl.getLeftX(), baseControl.getRightX());
        /*arm*/
        tamaruArm.setArmPower(-armControl.getLeftY());
    }
}