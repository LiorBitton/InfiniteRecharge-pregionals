package com.evergreen.robot.wpilib.commands;

import com.evergreen.robot.wpilib.Chassis;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Stop the robot
 */
public class Stop extends CommandBase {
    public Stop(){
        addRequirements(Chassis.getInstance());
    }
    @Override
    public void initialize() {
        Chassis.getInstance().getRightControllerGroup().set(0);
        Chassis.getInstance().getLeftControllerGroup().set(0);
    }
    @Override
    public boolean isFinished() {
        return true;
    }
}