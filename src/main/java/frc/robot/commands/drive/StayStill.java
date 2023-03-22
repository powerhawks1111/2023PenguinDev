package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Drivetrain;

public class StayStill extends CommandBase {
    public final Drivetrain m_drivetrain;
    private Timer m_timer = new Timer();
    private double delayTime = .2;
    /**
     * This is to stabailize the swerve drive during auto
     * @param drive
     */
    public StayStill (Drivetrain drive) {
        m_drivetrain = drive;
        
        addRequirements(m_drivetrain);
        
    }

    @Override
    public void initialize() {
        m_timer.start();
    }
    @Override
    public void execute () {
    

        m_drivetrain.drive(0, 0, (.0001 ), true, false); //final movement; sends drive values to swerve
    }

    @Override
    public boolean isFinished() {
        return m_timer.get() > delayTime;
    }
}
