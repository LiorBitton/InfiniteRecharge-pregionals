package com.evergreen.robot.wpilib.commands;

import com.evergreen.robot.wpilib.Utilites;
import com.evergreen.robot.wpilib.subsystem.Shooter;

import edu.wpi.first.wpilibj2.command.PIDCommand;

/**
 *pass the powecell to desierd {@link #m_distance} by using PID,
 * should not be used outside becuase you shold always first aim down
 * becuase the calculattion are about down angle <p>
 * <b>note:</b> if somehow we want to pass while we aim down change the code</p>
 * 
 */
class _PassPowerCell extends PIDCommand {
    
    public _PassPowerCell(double distance) {
        super(
            Shooter.getInstance().getThrowController(),
            Shooter.getInstance()::getThrowerSpeed,
            () -> getMotorSpeed(distance) ,
            Shooter.getInstance().m_thrower.m_motor::set,
            Shooter.getInstance().m_thrower);
            m_distance = distance;
    }
    /**
     * the desierd distance to shooting the ball in cm
     */
    private double m_distance;
    
    @Override
    public void initialize() {
        
        super.initialize();
    }
    public void setDistance( double distance){
        m_distance = distance; 
    }
    /**
     * calculate the desierd motor speed
     */
    public double getMotorSpeed(){
        double xGoal = m_distance;
        //hight goal = 0
        double angle = Shooter.getInstance().aimingDownAngle();
        double shootingTime =
        Math.sqrt((2*(-2*m_distance) * 
           Math.tan(angle) + Shooter.getInstance().SHOOTER_HEIGHT -0) /Utilites.GRAVITY_CONSTANT);
        double startXVelocity = 
           -m_distance/shootingTime;
        double startHightVelocity = 
    (2*(- Shooter.getInstance().SHOOTER_HEIGHT) - 
    Utilites.GRAVITY_CONSTANT*Math.pow(shootingTime, 2))
                        /2*shootingTime;

        return Math.sqrt(Math.pow(startXVelocity, 2) + Math.pow(startHightVelocity, 2));
    }
    public double getDistance(){
        return m_distance;
    }
    /**
     * used only for the constructor
     * 
     */
    private static double getMotorSpeed(double distance){
        double xGoal = distance;
        //hight goal = 0
        double angle = Shooter.getInstance().aimingDownAngle();
        double shootingTime =
        Math.sqrt((2*(-2*distance) * 
           Math.tan(angle) + Shooter.getInstance().SHOOTER_HEIGHT -0) /Utilites.GRAVITY_CONSTANT);
        double startXVelocity = 
           -distance/shootingTime;
        double startHightVelocity = 
    (2*(- Shooter.getInstance().SHOOTER_HEIGHT) - 
    Utilites.GRAVITY_CONSTANT*Math.pow(shootingTime, 2))
                        /2*shootingTime;

        return Math.sqrt(Math.pow(startXVelocity, 2) + Math.pow(startHightVelocity, 2));
    }
   

}