package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GrabberSubsystem;

public class PositionGrabber extends CommandBase{
    
    private GrabberSubsystem m_grabberSubsystem;
    private double m_position;

    /**
     * Position the grabber to the specified position
     * @param grabberSubsystem
     * @param position specified position
     */
    public PositionGrabber (GrabberSubsystem grabberSubsystem, double position) {
        m_grabberSubsystem = grabberSubsystem;
        m_position = position;
        addRequirements(m_grabberSubsystem);
    }

    @Override
    public void initialize () {}

    @Override
    public void execute () {
        m_grabberSubsystem.positionGrabber(m_position);
    }

    @Override
    public boolean isFinished () {
        return m_grabberSubsystem.closeToPosition();
    }

    @Override
    public void end(boolean interrupted) {

    } 
}
