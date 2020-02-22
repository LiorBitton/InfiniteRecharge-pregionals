package com.evergreen.robot.wpilib.commands;

import java.io.IOException;

import com.evergreen.robot.wpilib.Chassis;
import com.evergreen.robot.wpilib.DoubleArgCommand;
import com.evergreen.robot.wpilib.Chassis.TrajectoryOption;
import com.evergreen.robot.wpilib.subsystem.Shooter;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

/**
 * FollowTrajectory follow trajectory which drawn in pathweaver
 */
public class FollowTrajectory extends ParallelDeadlineGroup implements DoubleArgCommand{
    private boolean m_collectWhileMoving;
    /**
     * 
     * @param trajectory the trajectory to follow
     * @param collectWhileMoving put 1 if we wish to collect while we move, zero if we dont want, defualt zero
     */
    public FollowTrajectory(TrajectoryOption trajectory, double collectWhileMoving) throws IOException {
        super(new RamseteCommand(
            trajectory.getTrajectory(), 
            Chassis.getInstance().getOdometry()::getPoseMeters, 
            Chassis.getInstance().getRamseteController(), 
            Chassis.getInstance().getFeedForword(), 
            Chassis.getInstance().getKinematics(), 
            ()->Chassis.getInstance().getVelocity(),
            new PIDController(Chassis.getInstance().getDistanceKp(), Chassis.getInstance().getDistanceKi(), Chassis.getInstance().getDistanceKd()), 
            new PIDController(Chassis.getInstance().getDistanceKp(), Chassis.getInstance().getDistanceKi(), Chassis.getInstance().getDistanceKd()), 
            Chassis.getInstance()::setVoltage, 
            Shooter.getInstance())
        , new CommandBase[0]);
        m_collectWhileMoving = collectWhileMoving ==1;
    }
    /**
     * 
     * @param trajectory the trajectory to follow
     * @param collectWhileMoving are we wish to throw while we move
     */
    public FollowTrajectory(TrajectoryOption trajectory, boolean collectWhileMoving) throws IOException {
        super(new RamseteCommand(
            trajectory.getTrajectory(), 
            Chassis.getInstance().getOdometry()::getPoseMeters, 
            Chassis.getInstance().getRamseteController(), 
            Chassis.getInstance().getFeedForword(), 
            Chassis.getInstance().getKinematics(), 
            ()->Chassis.getInstance().getVelocity(),
            new PIDController(Chassis.getInstance().getDistanceKp(), Chassis.getInstance().getDistanceKi(), Chassis.getInstance().getDistanceKd()), 
            new PIDController(Chassis.getInstance().getDistanceKp(), Chassis.getInstance().getDistanceKi(), Chassis.getInstance().getDistanceKd()), 
            Chassis.getInstance()::setVoltage, 
            Shooter.getInstance())
        , new CommandBase[0]);
        m_collectWhileMoving = collectWhileMoving;
    }
    /**
     * 
     * @param collectWhileMoving put 1 if we wish to collect while we move, zero if we dont want, defualt zero
     */
    public void setCollectWhileMoving(double collectWhileMoving){
        m_collectWhileMoving = collectWhileMoving==1;
    }
    /**
     * 
     * @param collectWhileMoving are we wish to throw while we move
     */
    public void setCollectWhileMoving(boolean collectWhileMoving){
        m_collectWhileMoving = collectWhileMoving;
    }
    /**
     * 
     * @return 1 if we wish to collect while we move, zero if we dont want, defualt zero
     */
    public double getCollectWhileMovingDbl(){
        if(m_collectWhileMoving){
            return 1;
        }
        return 0;
    }
    /**
     * 
     * @return are we wish to throw while we move
     */
    public boolean getCollectWhileMovingBool(){
        return m_collectWhileMoving;
    }

    @Override
    public void setValue(double collectWhileMoving) {
       setCollectWhileMoving(collectWhileMoving);

    }

    @Override
    public double getValue() {
        return getCollectWhileMovingDbl();
    }
    

    
}