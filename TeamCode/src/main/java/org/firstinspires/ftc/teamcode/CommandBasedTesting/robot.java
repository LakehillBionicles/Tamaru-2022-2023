package org.firstinspires.ftc.teamcode.CommandBasedTesting;

import com.arcrobotics.ftclib.command.Robot;

public class robot extends Robot {

    // enum to specify opmode type
    public enum OpModeType { Tele, Auto}

    // the constructor with a specified opmode type
    public robot(OpModeType type) {
        if(type == OpModeType.Tele){
            // initialize teleop-specific scheduler
        } else {
            // initialize auto-specific scheduler
        }
    }

    public void initTele(){

    }

    public void initAuto(){

    }

}
