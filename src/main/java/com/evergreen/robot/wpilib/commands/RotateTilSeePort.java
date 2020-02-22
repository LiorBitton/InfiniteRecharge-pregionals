package com.evergreen.robot.wpilib.commands;

import com.evergreen.robot.wpilib.Chassis;
import com.evergreen.robot.wpilib.DoubleArgCommand;
import com.evergreen.robot.wpilib.Utilites;
import com.evergreen.robot.wpilib.subsystem.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * RotateTilSeePort rotate the robot until we see the power port
 */
public class RotateTilSeePort extends CommandBase implements DoubleArgCommand {
    /**
     * return true if we wish to rotate right, return false if we wish to rotate left
     */
    private double m_factor;
    private double m_speed;
    public RotateTilSeePort(boolean turnToRight, double speed){
        if(turnToRight){
            m_factor = 1;
        }else{
            m_factor = -1;
        }
        m_speed = speed;
        addRequirements(Chassis.getInstance());
    }
    public double getSpeed(){
        return m_speed;
    }
    public void setSpeed(double speed){
        m_speed = speed;
    }
    @Override
    public void initialize() {
        Chassis.getInstance().getRightControllerGroup().set(m_speed*m_factor);
        Chassis.getInstance().getLeftControllerGroup().set(m_speed*-m_factor);
    }
    @Override
    public boolean isFinished() {
        return Utilites.seePowerPort();
    }
    @Override
    public void end(boolean interrupted) {
        Chassis.getInstance().getRightControllerGroup().set(0);
        Chassis.getInstance().getLeftControllerGroup().set(0);
    }

    @Override
    public void setValue(double speed) {
        setSpeed(speed);

    }

    @Override
    public double getValue() {
      return getSpeed();
    }
}