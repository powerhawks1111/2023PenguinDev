package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GrabberSubsystem;

public class MoveGrabber extends CommandBase {
    /*
     * THIS FILE IS NOT BEING USED
     */
    /**
     * Move the grabber towards its open/closed position
     */
    private GrabberSubsystem m_grabberSubsystem;
    private double m_speed;
    public MoveGrabber(GrabberSubsystem subsystem, double speed) {
        m_grabberSubsystem = subsystem;
        m_speed = speed;
        addRequirements(m_grabberSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        m_grabberSubsystem.runGrabber(m_speed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_grabberSubsystem.runGrabber(0);
    }    
}
