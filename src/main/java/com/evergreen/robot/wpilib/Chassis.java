/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.evergreen.robot.wpilib;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.evergreen.robot.RobotMap.MotorPorts;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Chassis extends SubsystemBase {
  /**
   * Creates a new Chassis.
   */
private static Chassis m_instance;
  
  //declaring the front speed controllers (talon)
  private WPI_TalonSRX m_rightFront = new WPI_TalonSRX(MotorPorts.chassisRightFront);
  private WPI_TalonSRX m_leftFront = new WPI_TalonSRX(MotorPorts.chassisLeftFront);
 
  //declaring the gyro
  private Gyro m_gyro = new ADXRS450_Gyro();
  

  //declaring the other speed controllers
  private SpeedControllerGroup m_rightBack = new SpeedControllerGroup(new WPI_VictorSPX(MotorPorts.chassisRightBack), new WPI_VictorSPX(MotorPorts.chassisRightMiddle));
  private SpeedControllerGroup m_leftBack = new SpeedControllerGroup(new WPI_VictorSPX(MotorPorts.chassisLeftBack), new WPI_VictorSPX(MotorPorts.chassisLeftMiddle));
  
  private Command m_defaultDrive = new RunCommand(
    () -> drive(-Robot.getRightJoystick() * getSpeedModifier(), -Robot.getLeftJoystick() * getSpeedModifier()), 
    this);

  public Command getDefaultDrive() {
    return m_defaultDrive;
  }
  
  //creating pid componets for angle velocity and distance
  private double 
    ANGLE_KP = 0,
    ANGLE_KI = 0,
    ANGLE_KD = 0,
    ANGLE_TOLERANCE = 1,

    VELOCITY_KP = 0,
    VELOCITY_KI = 0,
    VELOCITY_KD = 0,
    VELOCITY_TOLERANCE = 1,
    
    DISTANCE_KP = 0,
    DISTANCE_KI = 0,
    DISTANCE_KD = 0,
    DISTANCE_TOLERANCE = 1;
  
  //creating pid controllers for angle velocity and distance
  private PIDController 
    m_anglePID = new PIDController(ANGLE_KP, ANGLE_KI, ANGLE_KD),
    m_velocityPID = new PIDController(VELOCITY_KP, VELOCITY_KI, VELOCITY_KD),
    m_distancePID = new PIDController(DISTANCE_KP, DISTANCE_KI, DISTANCE_KD);

  //Declaring Motion Profiling constants
  private final double 
    CHASSIS_WIDTH = 0.54,
    kS = 0,
    kV = 0,
    kA = 0,
    MAX_VELOCITY = 0,
    MAX_ACCELERATION = 0;  //m 
  //TODO check

  private DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(CHASSIS_WIDTH);

  private DifferentialDriveOdometry m_odometry;

  private SimpleMotorFeedforward m_feedorward = new SimpleMotorFeedforward(kS, kV, kA); 

  private TrajectoryConfig m_trajectoryConfig = new TrajectoryConfig(MAX_VELOCITY, MAX_ACCELERATION);

  private RamseteController m_ramseteController = new RamseteController();

  public DifferentialDriveKinematics getKinematics() {
    return m_kinematics;
  }

  public DifferentialDriveOdometry getOdometry(){
    return m_odometry;
  }

  public SimpleMotorFeedforward getFeedForword(){
    return m_feedorward;
  }

  public TrajectoryConfig getTrajectoryConfig(){
    return m_trajectoryConfig;
  }

  public RamseteController getRamseteController(){
    return m_ramseteController;
  }
  
  

  private Chassis() {
    //entering the PID componets into the prefernces
    m_rightBack.setInverted(true);
    m_rightFront.setInverted(true);
    m_gyro.calibrate();
    Preferences.getInstance().putDouble("Chassis/angle/KP", ANGLE_KP);
    Preferences.getInstance().putDouble("Chassis/angle/KI", ANGLE_KI);
    Preferences.getInstance().putDouble("Chassis/angle/KD", ANGLE_KD);
    Preferences.getInstance().putDouble("Chassis/angle/TOLERANCE", ANGLE_TOLERANCE);
    Preferences.getInstance().putDouble("Chassis/velocity/KP", VELOCITY_KP);
    Preferences.getInstance().putDouble("Chassis/velocity/KI", VELOCITY_KI);
    Preferences.getInstance().putDouble("Chassis/velocity/KD", VELOCITY_KD);
    Preferences.getInstance().putDouble("Chassis/velocity/TOLERANCE", VELOCITY_TOLERANCE);
    Preferences.getInstance().putDouble("Chassis/distance/KP", DISTANCE_KP);
    Preferences.getInstance().putDouble("Chassis/distance/KI", DISTANCE_KI);
    Preferences.getInstance().putDouble("Chassis/distance/KD", DISTANCE_KD);
    Preferences.getInstance().putDouble("Chassis/distance/TOLERANCE", VELOCITY_TOLERANCE);
    Preferences.getInstance().putDouble("Chassis/Speed", 0.8);
  }


  // creating a chassis object
  public static synchronized Chassis getInstance(){
    if (m_instance==null) m_instance = new Chassis();
    return m_instance;
  }
 
//rotating chassis to a choosen set point with pid
  public void rotateTo(double setpoint){//TODO: check
    
    m_anglePID.setSetpoint(setpoint);
    m_anglePID.setTolerance(ANGLE_TOLERANCE);

    while (Math.abs(m_anglePID.getPositionError()) > getPIDAngleTolerance()) {
      double speed = m_anglePID.calculate(m_gyro.getAngle());
      m_rightFront.set(speed);
      m_rightBack.set(speed);
      m_leftFront.set(-speed);
      m_leftBack.set(-speed);
    }
  }
  //sets same speed to all left motors
  public void setLeftSpeed(double speed){
    m_leftBack.set(speed);
    m_leftFront.set(speed);
  }
  // sets same speed to all right motors
  public void setRightSpeed(double speed){
    m_rightBack.set(speed);
    m_rightFront.set(speed);
  }
  
//returning the angle kp
  public double getAngleKp(){
    return Preferences.getInstance().getDouble("Chassis/distance/KD", ANGLE_KP);
  }
//returning the angle ki
  public double getAngleKi(){
    return ANGLE_KI;
  }
//returning the angle kd
  public double getAngleKd() {
    return ANGLE_KD;
  }
//returning the velocity kp
  public double getVelocityKp() {
    return VELOCITY_KP;
  }
//returning the velocity ki
  public double getVelocityKi() {
    return VELOCITY_KI;
  }
//returning the velocity kd
  public double getVelocityKd() {
    return VELOCITY_KD;
  }
//returning the distance kp
  public double getDistanceKp() {
    return DISTANCE_KP;
  }
//returning the distance ki
  public double getDistanceKi() {
    return DISTANCE_KI;
  }
//returning the distance kd
  public double getDistanceKd() {
    return DISTANCE_KD;
  }
//returning the distance from left sensor
  public double getLeftDistance() {
    return m_leftFront.getSelectedSensorPosition();
  } 
//returning the distance from right sensor
  public double getRightDistance() {
    return m_rightFront.getSelectedSensorPosition();
  }
//returning the speed of the right motors
 public double getRightVelocity(){
   return m_rightFront.getSelectedSensorVelocity();
 }
 //returning the speed of the left motor
 public double getLeftVelocity(){
   return m_leftFront.getSelectedSensorVelocity();
 }
 // returning the average speed
 public DifferentialDriveWheelSpeeds getVelocity() {
   return new DifferentialDriveWheelSpeeds(
     m_leftFront.getSelectedSensorVelocity(), 
     m_rightFront.getSelectedSensorVelocity());
 }

 //returning the average distance from both sensors
 public double getDistance(){
   return (getLeftDistance()+getRightDistance())/2;
 }
 //returning the angle tolerance
 private double getPIDAngleTolerance(){
   return ANGLE_TOLERANCE;
 }
 //returning the distance tolerance
 public double getPIDDistanceTolerance(){
  return DISTANCE_TOLERANCE;
}
//returning the velocity tolerance
private double getPIDVelocityTolerance(){
  return VELOCITY_TOLERANCE;
}
//returning the angle pid controller
public PIDController getAnglePID(){
  return m_anglePID;
}
//returning the distance pid controller
public PIDController getDistancePID(){
  return m_distancePID;
}
//returning the velocity pid controller
private PIDController getVelocityPID(){
  return m_velocityPID;
}
//returing the gyro object
public Gyro getGyro(){
  return m_gyro;
}
//returning the right talon mototr
public TalonSRX getRightTalonSRX(){
  return m_rightFront;
}
//returning the left talon motor
public TalonSRX getLefTalonSRX(){
  return m_leftFront;
}
//returning the right victor momtors
public SpeedControllerGroup getRightControllerGroup(){
  return m_rightBack;
}
//returning the left victor motors
public SpeedControllerGroup getLeftControllerGroup(){
  return m_leftBack;
}
//rotating the chassis in a certin speed
public void rotate(double speed){
  
  m_leftBack.set(speed);
  m_leftFront.set(speed);
  m_rightBack.set(-speed);
  m_rightFront.set(-speed);
}

//moving the entire chassis with a specific speed 
  public void move(double speed){
    
    m_leftBack.set(speed);
    m_leftFront.set(speed);
    m_rightBack.set(speed);
    m_rightFront.set(speed);
  } 
  
  //moving the chassis with a specific speed for right and left motors
  public void drive(double speedR, double speedL){
    m_leftBack.set(speedL);
    m_leftFront.set(speedL);
    m_rightFront.set(speedR);
    m_rightBack.set(speedR);
  }

  //TODO call this
  public void initOdometry(double x, double y) {
    m_odometry = new DifferentialDriveOdometry(
      getHeading(), new Pose2d(x, y, getHeading()));
  }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-m_gyro.getAngle());
  }
 
  public void setVoltage(double left, double right) {
    drive(left / 12, right / 12);
  }

  public RamseteCommand follow(TrajectoryOption trajectory) throws IOException {//have Not check yet
    return
      new RamseteCommand(
        trajectory.getTrajectory(), 
        m_odometry::getPoseMeters, 
        m_ramseteController, 
        m_feedorward, 
        m_kinematics, 
        this::getVelocity,
        new PIDController(getDistanceKp(), getDistanceKi(), getDistanceKd()), 
        new PIDController(getDistanceKp(), getDistanceKi(), getDistanceKd()), 
        this::setVoltage, 
        this);

    
  }


  public static enum TrajectoryOption {
    MOCK("Mock");

    String m_name;

    private TrajectoryOption(String name) {
      m_name = name;
    }

    public String getName() {
      return m_name;
    }

    public Trajectory getTrajectory() throws IOException {
      Path trajectoryJson = Filesystem.getDeployDirectory().toPath().resolve("paths/" + m_name + ".wpilib.json");
      return TrajectoryUtil.fromPathweaverJson(trajectoryJson);
      
    }
  }

  public double getSpeedModifier() {
    return 0.85;
  }
  

  
  @Override
  public void periodic() {
    // System.out.println(
    //   "LEFT JS: " + Rob
    //   "RIGHT JS: " + Robot.getRightJoystick());
  }
}
