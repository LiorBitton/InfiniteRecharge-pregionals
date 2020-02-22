package com.evergreen.robot.wpilib.commands;

import com.evergreen.robot.wpilib.Chassis;
import com.evergreen.robot.wpilib.DoubleArgCommand;

import edu.wpi.first.wpilibj2.command.PIDCommand;

public class RotateTo extends PIDCommand implements DoubleArgCommand {
//rotating to a certin point with pid
    //TODO: check if when we put negtive value it rotate left /right, opsite from positive values.
    public void setSetPoint(double setpoint){
        m_setpoint =()->setpoint;
    }
    public double getSetPoint(){
        return m_setpoint.getAsDouble();
    }
    public RotateTo(double setpoint) {
        
        super(
            Chassis.getInstance().getAnglePID(), //Controller
            () -> Chassis.getInstance().getGyro().getAngle() , //Mesurement Source
            () -> setpoint, //Setpoint Supplier
            Chassis.getInstance()::rotate, //Output Consumer
            Chassis.getInstance()); //Requirement
        
    }

    @Override
    public void setValue(double setpoint) {
        setSetPoint(setpoint);

    }

    @Override
    public double getValue() {
        return getSetPoint();
    }

}