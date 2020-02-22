package com.evergreen.robot.wpilib.commands;

import java.util.function.Supplier;

import com.evergreen.robot.wpilib.DoubleArgCommand;
import com.evergreen.robot.wpilib.Utilites;

import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * DriveToPowerPort by pid, not use circular motion, only turn drive turn.
 * should be used after {@link RotateTilSeePort} unless we sure that we are 
 * looking to power port
 */

 public class DriveToPowerPort extends SequentialCommandGroup implements DoubleArgCommand{
    /**
     * the desierd distance from power port after 
     * finshing the command
     */
    private double m_distanceFromPP;
    private Supplier <Double> m_xDistance;
    private Supplier <Double> m_TurnningAngle;
    /**
     * 
     * @param DistanceFromPPGoal the desierd distance from power port after 
     * finshing the command
     */
    public DriveToPowerPort(double distanceFromPPGoal){
        m_distanceFromPP = distanceFromPPGoal;
        m_xDistance = () ->Utilites.getXDistanceFromPowerPort() - m_distanceFromPP;
        m_TurnningAngle = () -> (Utilites.getPowerPortToAllinceStationAngle()* Math.atan(m_xDistance.get()/Utilites.getYDistanceFromPowerPort()));
    }
    public void setDistanceFromPPGoal(double distanceFromPPGoal){
        m_distanceFromPP = distanceFromPPGoal;
    }
    public double getDistanceFromPPGoal(){
        return m_distanceFromPP;
    }
    @Override
    public void schedule() {
       addCommands(
           new RotateTo(m_TurnningAngle.get())
            ,new MoveChassisTo(Utilites.Pythagoras(m_xDistance.get(), Utilites.getYDistanceFromPowerPort())),
            new RotateTo(-1*(90 - Utilites.getPowerPortToAllinceStationAngle() + m_TurnningAngle.get()))
            );

        super.schedule();
    }
    @Override
    public void schedule(boolean interruptible) {
        addCommands(
            new RotateTo(m_TurnningAngle.get())
             ,new MoveChassisTo(Utilites.Pythagoras(m_xDistance.get(), Utilites.getYDistanceFromPowerPort())),
             new RotateTo(-1*(90 - Utilites.getPowerPortToAllinceStationAngle() + m_TurnningAngle.get()))
             );
 
         super.schedule(interruptible);
    }
    @Override
    public void setValue(double distanceFromPPGoal) {
        setDistanceFromPPGoal(distanceFromPPGoal);

    }

    @Override
    public double getValue() {
       return getDistanceFromPPGoal();
    }
    
}