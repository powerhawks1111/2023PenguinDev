package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

public class ReleaseConeFast extends CommandBase {
    private GrabberSubsystem m_grabberSubsystem; 
    private Timer m_timer = new Timer();
    /**
     * Score the cone at the specified position
     * @param armSubsystem
     * @param height low, mid, or high
     */
    public ReleaseConeFast(GrabberSubsystem grabberSubsystem) {
        m_grabberSubsystem = grabberSubsystem;
        addRequirements(m_grabberSubsystem);
    }
    @Override
    public void initialize () {
        m_timer.start();
    }

    @Override 
    public void execute() {
        m_grabberSubsystem.runGrabber(-1);
    }

    @Override
    public boolean isFinished() {
        return m_timer.get() > 0.5;
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
