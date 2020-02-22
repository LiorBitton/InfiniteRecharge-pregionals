package com.evergreen.robot.wpilib.commands;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import com.evergreen.everlib.utils.PIDSettings;
import com.evergreen.robot.wpilib.Chassis;
import com.evergreen.robot.wpilib.Utilites;
import com.evergreen.robot.wpilib.subsystem.Shooter;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * TurnPowePortMiddle turn till the robot is infront the power port
 */
public class TurnPowePortInfront extends PIDCommand {
    private static TurnPowePortInfront m_instance = new TurnPowePortInfront();
    
    
    public TurnPowePortInfront() {
        super(new PIDController(Chassis.getInstance().getAngleKp(),Chassis.getInstance().getAngleKi(), Chassis.getInstance().getAngleKd()),
         Utilites::getPowerPortToRobotAngle, 0.0, Chassis.getInstance()::rotate, Shooter.getInstance());
        // TODO Auto-generated constructor stub
    }

    
}