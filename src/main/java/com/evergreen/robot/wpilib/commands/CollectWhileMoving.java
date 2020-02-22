package com.evergreen.robot.wpilib.commands;

import com.evergreen.robot.wpilib.Chassis;
import com.evergreen.robot.wpilib.Collector;
import com.evergreen.robot.wpilib.DoubleArgCommand;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;

/**
 * CollectWhileMoving collect the power cell while moving stright some distance in meters
 * finish when we reach the distance.
 */
public class CollectWhileMoving extends ParallelDeadlineGroup implements DoubleArgCommand {
    
    private MoveChassisTo m_move;
    public CollectWhileMoving(double distance) {
        super(Collector.getInstance().collectCmd());
        m_move = new MoveChassisTo(distance);
        setDeadline(m_move);
    }
    /**
     * 
     * @param distance distance in meters.
     */
    public void setDistance(double distance){
        m_move.setDistance(distance);
    }
    /**
     * 
     * @return distance in meters.
     */
    public double getDistance(){
        return m_move.getDistance();
    }

    @Override
    public void setValue(double distance) {
        setDistance(distance);
    }

    @Override
    public double getValue() {
        return getDistance();
    }
    
    
}