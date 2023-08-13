package org.firstinspires.ftc.robotcontroller.external;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: Linear OpMode", group="Linear Opmode")
//@Disabled
public class BasicOpMode_Linear extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor fpd = null;
    private DcMotor bpd = null;
    private DcMotor fsd = null;
    private DcMotor bsd = null;

    //@Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        fpd  = hardwareMap.get(DcMotor.class, "fpd");
        bpd = hardwareMap.get(DcMotor.class, "bpd");
        fsd = hardwareMap.get(DcMotor.class, "fsd");
        bsd = hardwareMap.get(DcMotor.class, "bsd");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        fpd.setDirection(DcMotor.Direction.REVERSE);
        bpd.setDirection(DcMotor.Direction.FORWARD);
        fsd.setDirection(DcMotor.Direction.REVERSE);
        bsd.setDirection(DcMotor.Direction.REVERSE);

        fpd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fsd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bpd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bsd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double frontPortPower;
            double backPortPower;
            double frontStarPower;
            double backStarPower;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            frontPortPower = Range.clip(drive + turn, -1.0, 1.0) ;
            backPortPower  = Range.clip(drive + turn, -1.0, 1.0) ;
            frontStarPower = Range.clip(drive - turn, -1.0, 1.0) ;
            backStarPower  = Range.clip(drive - turn, -1.0, 1.0) ;


            // Send calculated power to wheels
            /*fpd.setPower(frontPortPower);
            bpd.setPower(backPortPower);
            fsd.setPower(frontStarPower);
            bsd.setPower(backStarPower);*/

            fpd.setPower(drive);
            bpd.setPower(drive);
            fsd.setPower(drive);
            bsd.setPower(drive);

            // Show the elapsed game time and wheel power.
            //telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("fsd", fsd.getCurrentPosition());
            telemetry.addData("fpd", fpd.getCurrentPosition());
            telemetry.addData("bsd", bsd.getCurrentPosition());
            telemetry.addData("bpd", bpd.getCurrentPosition());
            telemetry.update();
        }
    }
}
