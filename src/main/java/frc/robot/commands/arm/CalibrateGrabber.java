package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GrabberSubsystem;

public class CalibrateGrabber extends CommandBase {
    /*
     * THIS FILE IS NOT BEING USED 
     */
    /**
     * Calibrate the grabber and set its home position
     */
    private GrabberSubsystem m_grabberSubsystem;
    public CalibrateGrabber (GrabberSubsystem grabberSubsystem) {
        m_grabberSubsystem = grabberSubsystem;
        addRequirements(m_grabberSubsystem);
    }

    @Override
    public void initialize () {}

    @Override
    public void execute() {
        if (!m_grabberSubsystem.returnCalibrated()) {
            m_grabberSubsystem.runGrabber(-.075);
        }
    }

    @Override
    public boolean isFinished () {
        return (m_grabberSubsystem.returnCalibrated()) || m_grabberSubsystem.checkGrabberAmps();
    }

    @Override
    public void end(boolean interrupted) {
        m_grabberSubsystem.runGrabber(0);
        m_grabberSubsystem.setCalibrated(true);
        m_grabberSubsystem.defineOpened();
    }
}
