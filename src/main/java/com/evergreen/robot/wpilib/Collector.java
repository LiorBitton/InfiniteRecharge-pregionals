/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.evergreen.robot.wpilib;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.evergreen.robot.RobotMap.MotorPorts;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Collector extends SubsystemBase {
  
  //creates the collector instance
  private static Collector m_instance;

  //motor speed constat
  private double COLLECTOR_SPEED = 0.8; 

  //creates the speed controllers for the system
  private SpeedController m_collectorMotor = new WPI_VictorSPX(MotorPorts.collector);

  /**
   * Collects Power Cells (by setting a speed to the collecting mechanism)
   */
  public CommandBase collectCmd(){
    return new RunCommand(() -> collect(getSpeed()), this) {
    @Override
    public void end(boolean interrupted) {
      collect(0);
    }  
  };}

  
  public CommandBase collectCmd(double speed){
    return new RunCommand(() -> collect(speed), this) {
      @Override
      public void end(boolean interrupted) {
        collect(0);
      };
  };
}
  /**
   * 
   * @param time in seconds
   * @return
   */
  public CommandBase collectForTime(long time){
    return new CommandBase() {
      @Override
      public void initialize() {
        collect(COLLECTOR_SPEED);
      }
      @Override
        public boolean isFinished() {
          try {
            Thread.sleep(time*1000);
          } catch (InterruptedException e) {
            e.printStackTrace();
            throw new RuntimeException();
          }
          return true;
        }
      @Override
        public void end(boolean interrupted) {
          collect(0);
        }
        };
  }
  

  /**
   * Creates a new Collector.
   */
  private Collector() {
    Preferences.getInstance().putDouble("Collector/Speed", COLLECTOR_SPEED);
  }

  /**
   * The get instance method for the collector instance.
   * */ 
  public static synchronized Collector getInstance() {
    if (m_instance == null) m_instance = new Collector();
    return m_instance;
  }

  //collection method
  public void collect(double m_speed) {
    m_collectorMotor.set(m_speed);
  }

  /**
   * Gets the collecting mechanism speed
   */
  public double getSpeed() { 
    return Preferences.getInstance().getDouble("Collector/Speed", COLLECTOR_SPEED);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
