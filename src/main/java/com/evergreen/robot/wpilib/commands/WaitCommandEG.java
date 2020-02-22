package com.evergreen.robot.wpilib.commands;

import com.evergreen.robot.wpilib.DoubleArgCommand;

import edu.wpi.first.wpilibj2.command.WaitCommand;

/**
 * WaitCommandEG
 */
public class WaitCommandEG extends WaitCommand implements DoubleArgCommand{
    private double m_second;
    public WaitCommandEG(double seconds) {
        super(seconds);
        m_second = seconds;
        // TODO Auto-generated constructor stub
    }
    public void setSeconds(double seconds){
        m_second = seconds;
    }
    public double getSeconds(){
        return m_second;
    }
    @Override
    public void setValue(double seconds) {
       setSeconds(seconds);

    }

    @Override
    public double getValue() {
        return getSeconds();
    }
    @Override
    public boolean isFinished() {
       return m_timer.hasPeriodPassed(m_second);
    }
    

    
}