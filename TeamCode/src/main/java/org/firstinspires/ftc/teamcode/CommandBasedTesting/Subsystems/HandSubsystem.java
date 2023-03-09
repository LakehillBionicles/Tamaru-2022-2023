package org.firstinspires.ftc.teamcode.CommandBasedTesting.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * HandSubsystem
 * 1 servo
 * Command grab() returns instant command to set hand position to closed
 * Command release() returns instant command to set hand position to open
 */
@Config
public class HandSubsystem extends SubsystemBase {
    private final Servo hand;
    public static double handOpen = .75;
    public static double handClosed = 0;

    public HandSubsystem(Servo servo) { this.hand = servo; }

    //public HandSubsystem(final HardwareMap hwMap, final String name) { hand = hwMap.get(Servo.class, name); }

    public Command grab() { return new InstantCommand(() -> hand.setPosition(handClosed), this); }

    public Command release() { return new InstantCommand(() -> hand.setPosition(handOpen), this); }

}
