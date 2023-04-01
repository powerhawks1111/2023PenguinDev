package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

public class SpinGrabberTimed extends CommandBase {
    private GrabberSubsystem m_grabberSubsystem; 
    private double m_speed;
    private Timer m_timer = new Timer();
    /**
     * Score the cone at the specified position
     * @param armSubsystem
     * @param height low, mid, or high
     */
    public SpinGrabberTimed(GrabberSubsystem grabberSubsystem, double speed) {
        m_grabberSubsystem = grabberSubsystem;
        m_speed = speed;
        addRequirements(m_grabberSubsystem);
    }
    @Override
    public void initialize () {
        m_timer.start();
    }

    @Override 
    public void execute() {
        m_grabberSubsystem.runGrabber(m_speed);
    }

    @Override
    public boolean isFinished() {
        return m_timer.get() > .2;
    }

    @Override
    public void end(boolean interrupt){
        m_grabberSubsystem.runGrabber(0);
        m_timer.stop();
        m_timer.reset();
        // m_grabberSubsystem.positionGrabber(0);
        // m_armSubsystem.deployPistons(true);
        // m_armSubsystem.positionArm(0);
    }
    
}
