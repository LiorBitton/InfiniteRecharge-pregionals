package com.evergreen.robot.wpilib.commands;

import com.evergreen.robot.wpilib.Chassis;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * ResetGyro reset the gyro
 */
public class ResetGyro extends CommandBase {
@Override
public void initialize() {
    Chassis.getInstance().getGyro().reset();
}
@Override
public boolean isFinished() {
    // TODO Auto-generated method stub
    return true;
}
    
}