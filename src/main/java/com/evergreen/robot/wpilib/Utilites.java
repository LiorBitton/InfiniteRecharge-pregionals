package com.evergreen.robot.wpilib;

import com.evergreen.robot.wpilib.subsystem.Shooter;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/**
 * Utilites
 */
//Reset when at zero zero
 public class Utilites {
    public static final double GRAVITY_CONSTANT = -9.80665;
    /**
     * the time between acccelarate the shooter and pass the ball to the shooter in sec.
     */
    //TODO: tune
    //may cause recursive
    public static final double TIME_TIL_SHOOTING = 0.3;
    public static CommandBase waitForShooting() {
       return new WaitCommand(TIME_TIL_SHOOTING);
    }
    //TODO: check
    public static final Pose2d POWER_PORT_POSE2D = new Pose2d(0, 2.38415,new Rotation2d(0));
    //TODO: fix ready for shoot, vison work angle getting
    private static NetworkTable m_table = NetworkTableInstance.getDefault().getTable("SmartDashboard");
       
    public static double getXDistanceFromPowerPort(){
        if(isVisionDistanceWork()&&isVisonAngleWork()){
        return getDirectDistanceFromPowerPort()* Math.sin(getPowerPortToAllinceStationAngle());
        }
        return getPose().getTranslation().getX()-POWER_PORT_POSE2D.getTranslation().getX();
    }
    public static double getYDistanceFromPowerPort(){
       if(isVisionDistanceWork()&&isVisonAngleWork()){
        return getDirectDistanceFromPowerPort()* Math.cos(getPowerPortToAllinceStationAngle());
       }
       return getPose().getTranslation().getY()-POWER_PORT_POSE2D.getTranslation().getY();
    }
    public static double getDirectDistanceFromPowerPort(){
       if(isVisionDistanceWork()){
        return m_table.getEntry("Distance").getDouble(0);
       }
       
       return getPose().getTranslation().getDistance(POWER_PORT_POSE2D.getTranslation());
       
    }
    
    /**
     * @return a angle in whatsapp 
     */
    public static double getPowerPortToAllinceStationAngle(){
        if(isVisonAngleWork()){
            return 90 -Chassis.getInstance().getGyro().getAngle()+getPowerPortToRobotAngle();
        }
        return Math.asin(getXDistanceFromPowerPort()/getDirectDistanceFromPowerPort());
    }
    /**
     * 
     * @return b angle, see whatsapp
     */
    public static double getPowerPortToRobotAngle(){ 
        if(isVisonAngleWork()){
        Preferences.getInstance().putDouble("another angle",SmartDashboard.getNumber("Angle", 0.3) );
        return SmartDashboard.getNumber("Angle", 360);
        }
        return Chassis.getInstance().getGyro().getAngle()- Math.atan(getYDistanceFromPowerPort()/getXDistanceFromPowerPort());
       
    }
    
    public static boolean seePowerPort(){
        return SmartDashboard.getBoolean("SeePowerPort", false);
    }
    public static Pose2d getPose(){
        return Chassis.getInstance().getOdometry().getPoseMeters();
    }
    /**
     * 
     * @param dX delta x
     * @param dY delta y
     * @return calculating hypotenuse according to pitagurs formua
     */
    public static double Pythagoras(double dX, double dY){
        return Math.sqrt(
            Math.pow(dX, 2) +
            Math.pow(dY, 2)
        );
    }
    /////////////////////////////is work///////////////////////////////
    /**
     * 
     * @return if we checked the passBySensor command and it work.
     */
    //TODO: check
    public static boolean isPassBySensorWork(){
        return false;
    }
    /**
     * @return we are want to shoot to the inner  true, else change it to false; 
     */
    //TODO: check
    public static boolean isShootingToInnerWork(){
        return true;
    }
    //TODO: check
    public static boolean isVisionDistanceWork(){
        if(isVisonWork()){
        return true;
        }
        return false;
    }
    //TODO: check
    public static boolean isVisonAngleWork(){
       
            return true;
    }
    //TODO: check
    public static boolean isVisonWork(){
        return true;
    }
    public static boolean isThrowerPIDWork(){
        return true;
    }
    /////////////////////////////////////////////////////////////////
    /**
     * 
     * @param accelerateCommand that used to accelerate the shooter
     * @return new command that first acelerate and then pass the ball to the shooter
     */
    public static CommandBase toFullShootingCommand(CommandBase accelerateCommand,CommandBase aimCommand){
        if (isPassBySensorWork()) {
            return new SequentialCommandGroup (
                aimCommand,
                accelerateCommand, 
                waitForShooting(),
                Storage.getInstance().passBySensorCmd());
        }

        return new SequentialCommandGroup(
                accelerateCommand,
                waitForShooting(),
                Storage.getInstance().passByTime());
    }
}