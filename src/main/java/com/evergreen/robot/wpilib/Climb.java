/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.evergreen.robot.wpilib;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.evergreen.robot.RobotMap.MotorPorts;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
  
  // creates an instance of Climb
  private static Climb m_instance;

  //creates speed constants
  private double CLIMB_UP_SPEED = 0.5;
  private double CLIMB_PULL_SPEED = 1; 
  private double CLIMB_PUSH_SPEED = -0.7; 
  private double ELEVATOR_DESCEND_SPEED = -0.55;
  private long DESCEND_TIME = 2000;

  //creates the speed controllers
  private SpeedController m_climbPull = new WPI_VictorSPX(MotorPorts.climbPull);
  public SpeedController getClimbPull(){
    return m_climbPull;
  }
  private SpeedController m_climbUp = new WPI_TalonSRX(MotorPorts.climbUp);
  public SpeedController getClimbUp(){
    return m_climbUp;
  }

  /**
   * Lifts the hook elevator
   */
  public CommandBase m_up(){

    return new RunCommand(() -> climbUp(getUpSpeed()), this) {
      @Override
      public void end(boolean interrupted) {
        m_climbUp.set(0);
        
      }
    };
  }


  public CommandBase getPush() {
    return 
      new RunCommand(() -> climbPull(CLIMB_PUSH_SPEED), this) {
        @Override
        public void end(boolean interrupted) {
          m_climbPull.set(0);
        }
      };
  }
  

  /**
   * Pulls up the robot
   */
  public CommandBase m_pull(){
   return new RunCommand(() -> climbPull(getPullSpeed()), this) {
    @Override
    public void end(boolean interrupted) {
      m_climbPull.set(0.0);
    }
  };}
  public CommandBase getClimbDown(){
    return new RunCommand(() -> climbUp(getDescendSpeed()), this) {
      @Override
      public void end(boolean interrupted) {
        m_climbUp.set(0.0);
      }
    };
  }

  /** Climb Constructor:
   * Creates a new Climb.
   */
  private Climb() {
    m_climbPull.setInverted(true);//positve values if pull.
    m_climbUp.setInverted(true);
    //uploads the speed constants to the shuffelboard
    Preferences.getInstance().putDouble("Climb/Elevator Speed", CLIMB_UP_SPEED);
    Preferences.getInstance().putDouble("Climb/Pull-Up Speed", CLIMB_PULL_SPEED);
    Preferences.getInstance().putDouble("Climb/Elevator Descend Speed", ELEVATOR_DESCEND_SPEED);
    Preferences.getInstance().putLong("Climb/Elevator Descend Time", DESCEND_TIME);
  }

  //creates get instance method
  public static synchronized Climb getInstance() {
    if (m_instance == null) m_instance = new Climb();
    return m_instance;
  }

  //creates the hook elevating method
  public void climbUp(double upSpeed) {
    m_climbUp.set(upSpeed);

  }

  //creates pulling up method
  public void climbPull(double pullSpeed) {
    m_climbPull.set(pullSpeed);
  }
  
  //ends climbing by setting all speeds to 0
  public void endClimb() {
    m_climbUp.set(0);
    m_climbPull.set(0);
  }

   /**
    * Gets the climber elevator speed
    */
   public double getUpSpeed(){
   return Preferences.getInstance().getDouble("Climb/Elevator Speed", CLIMB_UP_SPEED);
   }

   /**
    * Gets the pulling up speed
    */
   public double getPullSpeed() {
   return Preferences.getInstance().getDouble("Climb/Pull-Up Speed", CLIMB_PULL_SPEED);
   }

      /**
    * Gets the elevator descending speed
    */
   public double getDescendSpeed() {
     return Preferences.getInstance().getDouble("Climb/Elevator Descend Speed", ELEVATOR_DESCEND_SPEED); 
   }

      /**
    * Gets the elevator descending time
    */
   public long getDescendTime() {
     return Preferences.getInstance().getLong("Climb/Elevator Descend Time", DESCEND_TIME);
   }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
