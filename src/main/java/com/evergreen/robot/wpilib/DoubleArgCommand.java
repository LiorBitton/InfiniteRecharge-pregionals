package com.evergreen.robot.wpilib;

/**
 * DoubleArgCommand used for commands that use especially one argument like distance, time, speed etc. 
 */
public interface DoubleArgCommand {
public void setValue(double value);
public double getValue();
    
}