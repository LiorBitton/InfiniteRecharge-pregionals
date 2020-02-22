package com.evergreen.robot.wpilib.commands;
import com.evergreen.robot.wpilib.Chassis;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;

public class CircularMotion extends CommandBase {
    private NetworkTable table = NetworkTableInstance.getDefault().getTable("vision");
    //TODO: select if we want to make this class generic for every circular 
    //movement or only for go to the powerPort. 
    private double m_time,m_distanceBetween;
    public CircularMotion (double time,double distanceBetween){
        m_time = time;
        m_distanceBetween = distanceBetween;
        }
    private double m_arcsAngle = 180-2*getAngleBetween();

    private double getAngleBetween() {
        return  table.getEntry("angleFromHeaven").getDouble(0); //because Peleg is stupid (angle grom reflective)
    }
    private double getDistance(){
        return table.getEntry("distanceTo").getDouble(0); //distance between robot to light Reflective
    }
    private double getRadios(){
        return Math.sqrt((Math.pow(getDistance(), 2))/(2-2*Math.cos(m_arcsAngle)));
    }
    private double getVector(){
        return (2*Math.PI*getRadios()*m_arcsAngle)/(m_time*360);
    } 
    private double rightSpeed(){
        return getVector()*(getRadios()+m_distanceBetween/2);
    }
    private double leftSpeed(){
        return getVector()*(getRadios()-m_distanceBetween/2);
    }
    PIDCommand rightSpeed = new PIDCommand(Chassis.getInstance().getAnglePID(), () -> Chassis.getInstance().getRightVelocity(), rightSpeed(), Chassis.getInstance()::setRightSpeed);
    PIDCommand leftSpeed = new PIDCommand(Chassis.getInstance().getAnglePID(), () -> Chassis.getInstance().getLeftVelocity(), leftSpeed(), Chassis.getInstance()::setLeftSpeed);
    
    @Override
    public void initialize() {
        rightSpeed.initialize();
        leftSpeed.initialize();
    }

    @Override
    public void execute() {
        rightSpeed.execute();
        leftSpeed.execute();
    }
    @Override
    public boolean isFinished() {
        return (getAngleBetween() == 0);
    }
    @Override
    public void end(boolean interrupted) {
        Chassis.getInstance().setLeftSpeed(0);
        Chassis.getInstance().setRightSpeed(0);
    }
}