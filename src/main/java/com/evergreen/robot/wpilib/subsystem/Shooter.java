package com.evergreen.robot.wpilib.subsystem;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.evergreen.robot.RobotMap;
import com.evergreen.robot.wpilib.Utilites;
import com.evergreen.robot.wpilib.commands.PassPowerCell;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
/**
 * Shooter subsystem, basically divided to two Subsystem:
 * <ul>
 * <li> <b>the aimer</b> - the part that aim the thrower</li>
 * <li> <b>the thrower</b> - the two wheels which shoot the power cell</li>
 * </ul>
 */

public class Shooter extends SubsystemBase implements RobotMap {
    private static Shooter m_shooter;
    ////////////////////////sub Subsystem/////////////////////////////////////////
    // TODO:put correct SpeedController Type
     /**
     * the part that aim the thrower
     */
    public Aimer m_aimer = new Aimer();
     /**
     * the two wheels which shoot the power cell
     */
    public Thrower m_thrower = new Thrower();
    
    /**
     * the part that aim the thrower
     */
    public class Aimer extends SubsystemBase{
        // TODO:put correct SpeedController Type
        
        public WPI_TalonSRX m_motor = new WPI_TalonSRX(MotorPorts.aimer);
        public DigitalInput m_downSwitch = new DigitalInput(DigitalPorts.aimerSwitch);
        
    }

    /**
     * the two wheels which shoot the power cell
     */
    public class Thrower extends SubsystemBase{
        // TODO:put correct SpeedController Type
        public SpeedController m_motor = new WPI_VictorSPX(MotorPorts.thrower);
        public Encoder m_encoder = new Encoder(DigitalPorts.throwerEncoderA, DigitalPorts.throwerEncoderB);
    }

    public static SpeedController m_throwerMotor = new WPI_TalonSRX(MotorPorts.thrower);

    //////////////////////////////////////////////////////////////////////////
    /**
     * shooter highet in cm
     */
    public static final double SHOOTER_HEIGHT = 29;
    /**
     * if the aimer is at the upper position return true if it at the lower position
     * return false if between return last position
     */
    // TODO: find start position
    public boolean m_atUpperPosition = true;

    //////////////////////// tuneable values//////////////////////////////////////
    // TODO: tune the values
    public static double aimingUpAngle() {
        return Preferences.getInstance().getDouble("aimingUpAngle", 30);
    }

    public static double aimingDownAngle() {
        return Preferences.getInstance().getDouble("aimingDownAngle", 10);
    }

    public double aimerKp() {
        return Preferences.getInstance().getDouble("aimerKp", 0);
    }

    public double aimerKi() {
        return Preferences.getInstance().getDouble("aimerKi", 0);
    }

    public double aimerKd() {
        return Preferences.getInstance().getDouble("aimerKd", 0);
    }

    public double throwerKp() {
        return Preferences.getInstance().getDouble("throwerKp", 0);
    }

    public double throwerKi() {
        return Preferences.getInstance().getDouble("throwerKi", 0);
    }

    public double throwerKd() {
        return Preferences.getInstance().getDouble("throwerKd", 0);
    }

    public double aimerAnglePerPulse() {
        return Preferences.getInstance().getDouble("aimer/anglePerPulse", 1);
    }

    public double throwerDistancePerPulse() {
        return 0.5 * Math.PI;
    }

    /**
     * 
     * @return the speed that used to drop the ball
     */
    public double droppingSpeed() {
        return Preferences.getInstance().getDouble("thrower/droppingSpeed", 0.06);
    }

    ////////////////////////////////////////////////////////////////////////////
    //////////////////////////// encoders output/////////////////////////////////
    public double getAimerAngle() {
        return m_aimer.m_motor.getSelectedSensorPosition() * aimerAnglePerPulse();
    }

    public double getThrowerSpeed() {
        return m_thrower.m_encoder.getRate();
    }
    ////////////////////////////////////////////////////////////////////////////

    //////////////////////////// pid controllers/////////////////////////////////
    private PIDController m_aimController = new PIDController(aimerKp(), aimerKi(), aimerKd());

    private PIDController m_throwController = new PIDController(throwerKp(), throwerKi(), throwerKd());
    ////////////////////////////////////////////////////////////////////////////

    //////////////////////////// pid controllers
    //////////////////////////// getters/////////////////////////////////
    public PIDController getAimController() {
        return m_aimController;
    }

    public PIDController getThrowController() {
        return m_throwController;
    }
    ////////////////////////////////////////////////////////////////////////////

    /////////////////////////// commands////////////////////////////////////////////////

    private CommandBase m_aimUp() {
        return new CommandBase() {
            double m_start;
            double m_difference;

            public void initialize() {
                m_start = System.currentTimeMillis();
            }

            @Override
            public void execute() {
                m_aimer.m_motor.set(0.7);
                SmartDashboard.putNumber("START", m_start);

                SmartDashboard.putNumber(
                    "CURRENT", System.currentTimeMillis());
                
                m_difference = System.currentTimeMillis() - m_start;
                
                SmartDashboard.putNumber("DIFFERENCE", m_difference);
            }

            @Override
            public boolean isFinished() {
                return m_difference >= 190;    
            }

            @Override
            public void end(boolean interrupted) {
                m_aimer.m_motor.set(0);
            }
        };
    }
    

    private CommandBase m_aimDown() {
        return new CommandBase() {
           
            @Override
            public void execute() {
                m_aimer.m_motor.set(-0.3);
            }

            @Override
            public boolean isFinished() {
                return m_aimer.m_downSwitch.get();
            }

            @Override
            public void end(boolean interrupted) {
            // Created weird bang bang
             
                m_aimer.m_motor.set(0);
            }
        };
        }
    private CommandBase m_aimToggle() {
        return new PIDCommand(m_aimController, this::getAimerAngle, () -> {
            if (m_atUpperPosition) {
                return aimingDownAngle();
            }
            return aimingUpAngle();
        }, m_aimer.m_motor::set, m_aimer) {
            public void end(boolean interrupted) {
                super.end(interrupted);
                m_atUpperPosition = !m_atUpperPosition;
            }
        };
    }

    /**
     * stops the thrower
     */
    private CommandBase m_stopThrower() {
        return new InstantCommand(() -> m_thrower.m_motor.set(0), m_thrower);
    }

    /**
     * accelerate for shooting to inner/outer according to
     * {@link Utilites#isShootingToInnerWork()} .
     * 
     */
    private CommandBase m_accelerateUp() {
        return new PIDCommand(m_aimController, this::getThrowerSpeed, () -> {
            if (Utilites.isShootingToInnerWork()) {
                return PowerPorts.INNER.motorSpeed;
            }
            return PowerPorts.OUTER.motorSpeed;
        }, m_thrower.m_motor::set, m_thrower);
    }

    /**
     * acccelerate for shooting to the bootom.
     */
    private CommandBase m_accelerateBottom() {
        return new PIDCommand(m_aimController, this::getThrowerSpeed, PowerPorts.BOTTOM.motorSpeed,
                m_thrower.m_motor::set, m_thrower);
    }

    /**
     * if aimer at upper accelerate for shooting to inner/outer according to
     * {@link Utilites#isShootingToInnerWork()} if aimer at the lower position
     * acccelerate for shooting to the bootom.
     */
    private CommandBase m_accelerteTothrow() {
        return new PIDCommand(m_aimController, this::getThrowerSpeed,()->{ 
        if (m_atUpperPosition) {
            if(Utilites.isShootingToInnerWork()){

            return  PowerPorts.INNER.motorSpeed;
            }
                return PowerPorts.OUTER.motorSpeed;
            }

            return PowerPorts.BOTTOM.motorSpeed;
        }, m_thrower.m_motor::set, m_thrower);
    }

    /**
     * fully(aim while accelerate and then passToShooter) shoot to outer or inner
     * port according to {@link Utilites#isShootingToInnerWork}
     */
    private CommandBase shootToUpper() {
        return Utilites.toFullShootingCommand(m_accelerateUp(), m_aimUp());
    }

    /**
     * fully(aim while accelerate and then passToShooter) shoot to bottom port
     */
    private CommandBase shootToBottom() {
        return Utilites.toFullShootingCommand(m_accelerateBottom(), m_aimDown());
    }

    /**
     * fully(aim while accelerate and then passToShooter) drop the ball and leave it
     * for our aliince members.
     */
    // TODO: find how many time take to the robot to dropm_thrower
    private CommandBase drop = Utilites
            .toFullShootingCommand(new InstantCommand(() -> m_thrower.m_motor.set(droppingSpeed())) {
                public void end(boolean interrupted) {
                    m_thrower.m_motor.set(0);
                }
            }, new WaitUntilCommand(0));

    /////////////////////////////////////////////////////////////////////////

    ///////////////////////// commands getters///////////////////////////////
    /**
     * s
     * 
     * @return {@link Shooter#m_aimUp}
     */
    public CommandBase getAimUp() {
        return m_aimUp();
    }

    /**
     * 
     * @return {@link Shooter#m_aimDown}
     */
    public CommandBase getAimDown() {
        return m_aimDown();
    }

    /**
     * 
     * @return {@link Shooter#m_aimToggle}
     */
    public CommandBase getAimToggle() {
        return m_aimToggle();
    }

    /**
     * 
     * @return {@link m_stopThrower} - stops the thrower
     */
    public CommandBase getStopThrower() {
        return m_stopThrower();
    }

    /**
     * 
     * @return {@link Shooter#m_accelerateUp} - accelerate for shooting to
     *         inner/outer according to {@link Utilites#isShootingToInnerWork()}
     */
    public CommandBase getAccelerateUp() {
        return m_accelerateUp();
    }

    /**
     * 
     * @return {@link Shooter#m_accelerateBottom} - acccelerate for shooting to the
     *         bootom
     */
    public CommandBase getAccelerateBottom() {
        return m_accelerateBottom();
    }

    /**
     * 
     * @return {@link Shooter#m_accelerteTothrow} - if aimer at upper accelerate for
     *         shooting to inner/outer according to
     *         {@link Utilites#isShootingToInnerWork()} if aimer at the lower
     *         position acccelerate for shooting to the bootom.
     */
    public CommandBase getAccelerateToThrow() {
        return new CommandBase() {
            @Override
            public void execute() {
                m_thrower.m_motor.set(0.8);
            }

            @Override
            public void end(boolean interrupted) {
                m_thrower.m_motor.set(0);
            }
        };
    }

    /**
     * 
     * @return {@link Shooter#shootToUpper} - fully(aim while accelerate and then
     *         passToShooter) shoot to outer or inner port according to
     *         Utilites.isShootingToInnerWork
     */
    public CommandBase getShootToUpper() {
        return shootToUpper();
    }

    /**
     * 
     * @return {@link Shooter#shootToBottom} - fully(aim while accelerate and then
     *         passToShooter) shoot to bottom port
     */
    public CommandBase getShootToBottom() {
        return shootToBottom();
    }

    /**
     * 
     * @return {@link Shooter#drop} - fully(aim while accelerate and then
     *         passToShooter) drop the ball and leave it for our aliince members.
     */
    public CommandBase getDrop() {
        return drop;
    }

    /**
     * 
     * @return {@link Shooter#m_pass} - fully (aim while accelerate and then
     *         passToShooter) pass the power cell to the desierd distance
     */
    public static CommandBase getPass() {
        return new PassPowerCell(700);
    }

    /////////////////////////////////////////////////////////////////
    /**
     * update the distance that we want to throw to
     */
    public void updatePassDistance() {
        // TODO decide how to implemnt by m_pass.setDistance()
    }

    /**
     * Construct new Shooter and at the values to the shufflBoard
     */
    private Shooter() {
        m_aimer.m_motor.setInverted(true);
        Preferences.getInstance().putDouble("aimingUpAngle", 30);
        Preferences.getInstance().putDouble("aimingDownAngle", 15);
        Preferences.getInstance().putDouble("aimerKp", 0);
        Preferences.getInstance().putDouble("aimerKi", 0);
        Preferences.getInstance().putDouble("aimerKd", 0);
        Preferences.getInstance().putDouble("throwerKp", 0);
        Preferences.getInstance().putDouble("throwerKi", 0);
        Preferences.getInstance().putDouble("throwerKd", 0);
        Preferences.getInstance().putDouble("thrower/speedGoal", 0.1);
        Preferences.getInstance().putDouble("aimer/anglePerPulse", 1);
        Preferences.getInstance().putDouble("thrower/distancePerPulse", 1);
        Preferences.getInstance().putDouble("thrower/droppingSpeed", 0.06);
        // m_aimer.m_motor.config(aimerAnglePerPulse());
        m_thrower.m_encoder.setDistancePerPulse(throwerDistancePerPulse());

    }

    public static synchronized Shooter getInstance() {
        if (m_shooter == null)
            m_shooter = new Shooter();
        return m_shooter;
    }

    /**
     * Present the power ports and their constant in cm for calculate the motorspeed
     */
    public static enum PowerPorts {
        // sizes in cm
        INNER(0, 249, aimingUpAngle(), "circle"), OUTER(74.295, 249, aimingUpAngle(), "hexagon"),
        BOTTOM(74.295, 46, aimingDownAngle(), "rectangle");

        public final double X_GOAL;
        public final double HIGHT_GOAL;
        public double m_angle;
        public String m_shape;
        public double startXVelocity;
        public double startHightVelocity;
        public double shootingTime;
        public double motorSpeed;

        /**
         * calculate the varibles and the motorSpeed
         * 
         */
        private PowerPorts(double xGoal, double hightGoal, double angle, String shape) {
            HIGHT_GOAL = hightGoal;
            X_GOAL = xGoal;
            m_shape = shape;
            m_angle = angle;
            // FIXME: check if this formula is like the orignal, if change here change
            // in PassPowerCell
            shootingTime = Math.sqrt((-2 * Utilites.getXDistanceFromPowerPort() * Math.tan(m_angle)
                    + Shooter.SHOOTER_HEIGHT - HIGHT_GOAL) / Utilites.GRAVITY_CONSTANT);
            startXVelocity = (-Utilites.getXDistanceFromPowerPort()) / shootingTime;

            startHightVelocity = (2 * (HIGHT_GOAL - Shooter.SHOOTER_HEIGHT)
                    - Utilites.GRAVITY_CONSTANT * Math.pow(shootingTime, 2)) / 2 * shootingTime;
            motorSpeed = Math.sqrt(Math.pow(startXVelocity, 2) + Math.pow(startHightVelocity, 2));

        }

        
        
    }
    public final double SHOOTER_KS = 1.45, SHOOTER_KV = 0.000258, SHOOTER_KA = 8.41e-5, WANTED_SPEED = 0.5;

    public double shooterSpeedToPercentage(double speed, double acceleration) {
        double voltage = 8 / 5 * (SHOOTER_KS * Math.signum(speed) + SHOOTER_KV * speed + SHOOTER_KA * acceleration);
        SmartDashboard.putNumber("VOLTAGE", voltage);
        return voltage / 12; // Percentage
    }
    
    public CommandBase getShooterSpeedControl() {
        return new CommandBase() {
            @Override
            public void initialize() {
                m_throwerMotor.set(shooterSpeedToPercentage(0.5, 0));
            }
        };
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("TICKS", m_aimer.m_motor.getSelectedSensorPosition());
    }
}
