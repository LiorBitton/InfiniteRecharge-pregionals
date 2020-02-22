package com.evergreen.robot.wpilib.commands;

/**
 * DriveToNoStop drive without stop while is.
 */
public class DriveToNoStop extends MoveChassisTo {

    public DriveToNoStop(double distance) {
        super(distance);
    }
    @Override
    public void end(boolean interrupted) {
        
    }
    
}