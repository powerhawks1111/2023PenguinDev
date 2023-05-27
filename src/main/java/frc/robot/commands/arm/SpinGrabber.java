package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

public class SpinGrabber extends CommandBase {
    private GrabberSubsystem m_grabberSubsystem; 
    private double m_speed;
    /**
     * Spin the grabber at a specified speed
     * @param grabberSubsystem
     * @param speed -1 to +1 (positive intakes, negative outtakes)
     */
    public SpinGrabber(GrabberSubsystem grabberSubsystem, double speed) {
        m_grabberSubsystem = grabberSubsystem;
        m_speed = speed;
        addRequirements(m_grabberSubsystem);
    }
    @Override
    public void initialize () {}

    @Override 
    public void execute() {
        m_grabberSubsystem.runGrabber(m_speed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupt) {
        m_grabberSubsystem.runGrabber(0);
        // m_grabberSubsystem.positionGrabber(0);
        // m_armSubsystem.deployPistons(true);
        // m_armSubsystem.positionArm(0);
    }
    
}
