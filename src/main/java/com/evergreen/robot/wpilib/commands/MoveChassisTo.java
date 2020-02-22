package com.evergreen.robot.wpilib.commands;

import com.evergreen.robot.wpilib.Chassis;
import com.evergreen.robot.wpilib.DoubleArgCommand;

import edu.wpi.first.wpilibj2.command.PIDCommand;
//Change m_distance
/**
 * MoveChassisTo
 */
public class MoveChassisTo extends PIDCommand implements DoubleArgCommand {

    private double m_distance;
    public void setDistance(double distance){
        m_setpoint = () -> (distance);
        m_distance =distance;
    }
    public double getDistance(){
        return m_distance;
    }
    
    //moving chassis to a certin distance with pid
    public MoveChassisTo(double distance) {
        super(Chassis.getInstance().getDistancePID(), 
        () -> Chassis.getInstance().getDistance(), () -> distance, Chassis.getInstance()::move, Chassis.getInstance());
        m_distance = distance;
    }
    //TODO: mybe we should check if we are at setPoint three time and not one time
    @Override
    public boolean isFinished() {
        return ((getController().getPositionError())<=Chassis.getInstance().getPIDDistanceTolerance());
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