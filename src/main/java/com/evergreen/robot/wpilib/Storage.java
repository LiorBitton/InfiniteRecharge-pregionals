/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.evergreen.robot.wpilib;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.evergreen.robot.RobotMap.AnalogPorts;
import com.evergreen.robot.RobotMap.DigitalPorts;
import com.evergreen.robot.RobotMap.MotorPorts;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Storage extends SubsystemBase {

  // Declares the Storage single instance
  private static Storage m_instance;

  // The passing speed, time and minimum empty distance constants
  private double PASS_SPEED = 0.8;
  private long PASS_TIME = 2000; //Notice the time is in milliseconds
  private double MIN_EMPTY_DIST; //Minimum Distance in which Storage is Empty, in mm
  //TODO: find the correct value of minimum distance

  //New speed controller and ultrasonic sensor for passing the power cells.
  private SpeedController m_passMotor = new WPI_VictorSPX(MotorPorts.passer);
   private AnalogInput ultrasonic = 
     new AnalogInput(AnalogPorts.storageUltrasonic);
  // TODO Y 2 ports needed
  /**
   * Passes a Power Cell to the Shooter, stops after a fixed amount of time.
   */
  public CommandBase passByTime() {
    return new CommandBase() {
      @Override
        public void initialize() {
          m_passMotor.set(getSpeed());
        }
      @Override
        public boolean isFinished() {
          try {
            Thread.sleep(getTime());
          } catch (InterruptedException e) {
            e.printStackTrace();
            throw new RuntimeException();
          }
          return true;
        }
          @Override
        public void end(boolean interrupted) {
          m_passMotor.set(0);
        }
        };
      
  };
  

  /**
   * Passes a Power Cell to the Shooter, stops by the Ultrasonic sensor signals.
   */
  public CommandBase passBySensorCmd(){
    return new RunCommand(() -> passBySensor(getSpeed(), getUltrasonicDistance()), this) {
    
      @Override 
    public void end(boolean interrupted) {
      m_passMotor.set(0);
    }
  };
}
  
  /**
   * Creates a new Storage.
   */
  private Storage() {
    Preferences.getInstance().putDouble("Storage/Passing Speed", PASS_SPEED);
    Preferences.getInstance().putLong("Storage/Passing Time", PASS_TIME);
    Preferences.getInstance().putDouble("Storage/Minimum Empty Distance", MIN_EMPTY_DIST); 
    //Minimum Distance in which Storage is Empty
  }

  /**
   * Gets the Storage single instance.
   */
  public static synchronized Storage getInstance() {
    if (m_instance == null)
      m_instance = new Storage();
    return m_instance;
  }

  /**
   * Sets the passing motor to input speed.
   * 
   * @throws InterruptedException
   */
  public void passByTime(double speed, long time) {
    m_passMotor.set(speed);
    try {
      Thread.sleep(time);
    } catch (InterruptedException e) {
      e.printStackTrace();
      throw new RuntimeException();
    }
    m_passMotor.set(0);
  }
  public CommandBase getPass(){
    return new CommandBase() {
      @Override
      public void initialize() {
        m_passMotor.set(getSpeed());
      }
      @Override
        public void end(boolean interrupted) {
          m_passMotor.set(0);
        }
    };
  }
  //TODO: check if work
  public void passBySensor(double m_speed, double dist) {
    if ((dist <= MIN_EMPTY_DIST) && (dist != 0)) {
      m_passMotor.set(m_speed);
    }
    if (dist > MIN_EMPTY_DIST) m_passMotor.set(0);
  }


  /**
   * Gets the storage passing motor speed
   */
  public double getSpeed() {
    return Preferences.getInstance().getDouble("Storage/Passing Speed", PASS_SPEED);
  }

  /**
   * Gets the storage passing motor time (in milliseconds).
   */
  public long getTime() {
    return Preferences.getInstance().getLong("Storage/Passing Time", PASS_TIME);

  }

  /**
   * Gets the Ultrasonic value (in mm).
   */
   public double getUltrasonicDistance() {
    return ultrasonic.getVoltage();
  }

  /**
   * Gets the minimum distance in which the storage is considered empty (in mm).
   */
  public double getMinEmptyDistance() {
    return Preferences.getInstance().getDouble("Storage/Minimum Empty Distance", MIN_EMPTY_DIST);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
