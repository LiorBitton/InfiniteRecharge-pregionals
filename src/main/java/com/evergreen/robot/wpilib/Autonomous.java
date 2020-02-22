package com.evergreen.robot.wpilib;

import java.io.IOException;
import java.util.function.Supplier;

import com.evergreen.robot.wpilib.Chassis.TrajectoryOption;
import com.evergreen.robot.wpilib.commands.CollectWhileMoving;
import com.evergreen.robot.wpilib.commands.DriveToNoStop;
import com.evergreen.robot.wpilib.commands.DriveToPowerPort;
import com.evergreen.robot.wpilib.commands.FollowTrajectory;
import com.evergreen.robot.wpilib.commands.MoveChassisTo;
import com.evergreen.robot.wpilib.commands.RotateTilSeePort;
import com.evergreen.robot.wpilib.commands.RotateTo;
import com.evergreen.robot.wpilib.commands.Stop;
import com.evergreen.robot.wpilib.commands.TurnPowePortInfront;
import com.evergreen.robot.wpilib.commands.WaitCommandEG;
import com.evergreen.robot.wpilib.subsystem.Shooter;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/**
 * Autonomous class used for the 
 * <b>note:</b> DriveXDistanceFromPowerPort should bee use after
 * turnTilSeePowerPort unless we sure that the robot is in front of the power port
 */
public class Autonomous extends SequentialCommandGroup {
    /**
     * the number of options that we wish to have.
     */
    public static final int OPTIONS_NUMBER = 1;
    private SendableChooser<CommandBase>[] m_options = new SendableChooser[OPTIONS_NUMBER];
    private Supplier<Double>[] m_arguments = new Supplier[OPTIONS_NUMBER];
    private CommandBase[] m_commands;
    private static Autonomous m_instance;

    public static synchronized Autonomous getInstance() {
        if (m_instance == null) m_instance = new Autonomous();
        return m_instance;
    }
    
    //add collect while moving,
    private Autonomous() {
        m_commands = new CommandBase[OPTIONS_NUMBER];
        for (int i = 0; i < OPTIONS_NUMBER; i++) {
            final int j = i;
            Preferences.getInstance().putDouble("arg" + i, 0);
            m_arguments[i] = () -> Preferences.getInstance().getDouble("arg" + j, 0);
            m_options[i] = new SendableChooser<CommandBase>();
            m_options[i].setDefaultOption("wait #" + i, new WaitCommandEG(m_arguments[i].get()));
            m_options[i].addOption("driveStraight #" + i, new MoveChassisTo(m_arguments[i].get()));
            m_options[i].addOption("driveXdistanceWithoutStopping #", new DriveToNoStop(m_arguments[i].get()));
            m_options[i].addOption("driveXdistanceWhileCollecting #", new CollectWhileMoving(m_arguments[i].get()));
            m_options[i].addOption("riveXDistanceFromPowerPort #"+ i, new DriveToPowerPort(m_arguments[i].get()) );         
            //turn right by defualt if we want to turn left put negative value;
            m_options[i].addOption("rotate #" + i, new RotateTo(m_arguments[i].get()));
            //turn right by defualt if we want to turn left put negative value;
            m_options[i].addOption("turnTilSeePowerPort" +i, new RotateTilSeePort(true, m_arguments[i].get()));
            // m_options[i].addOption("turn until the power port is infront"+i, TurnPowePortInfront.getInstance());  
            m_options[i].addOption("Stop"+i, new Stop());
            //TODO: add circular move;
            m_options[i].addOption("shootToUpper" +i, Shooter.getInstance().getShootToUpper());
            m_options[i].addOption("shootToBottom"+ i, Shooter.getInstance().getShootToBottom());
            m_options[i].addOption("drop" + i, Shooter.getInstance().getDrop());
            m_commands[i] = m_options[i].getSelected();
            for (TrajectoryOption option : TrajectoryOption.values()) {
            
            try {
                m_options[i].addOption("follow " + option.getName() + i, new FollowTrajectory(option,m_arguments[i].get()));
            }

            catch (IOException e) {
                m_options[i].addOption("trajectory name NOT FOUND", new WaitCommand(0));
            }

            SmartDashboard.putData(m_options[i]);
        }
    }
    }
    
    @Override
    public void schedule() {
        addCommands(m_commands);
        super.schedule();
    }
    @Override
    public void schedule(boolean interruptible) {
        addCommands(m_commands);
        super.schedule(interruptible);
    }
    
    public void update() {
        for (int i = 0; i < OPTIONS_NUMBER; i++) {
            m_commands[i] = m_options[i].getSelected();
            if (m_options[i] instanceof DoubleArgCommand) {
                ((DoubleArgCommand) (m_options[i].getSelected())).setValue(m_arguments[i].get());
            }
        }
    }
   
    public CommandBase[] getCurrentCommands(){
        return m_commands;
    }
}