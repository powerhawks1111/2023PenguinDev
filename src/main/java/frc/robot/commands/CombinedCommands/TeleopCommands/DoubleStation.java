package frc.robot.commands.CombinedCommands.TeleopCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

public class DoubleStation extends CommandBase {
    private ArmSubsystem m_armSubsystem;
    private GrabberSubsystem m_grabberSubsystem;
    public DoubleStation(ArmSubsystem armSubsystem, GrabberSubsystem grabberSubsystem) {
        m_armSubsystem = armSubsystem;
        m_grabberSubsystem = grabberSubsystem;
    }

    @Override
    public void execute() {
        m_armSubsystem.positionArm(1.389); // 1.389 
        m_grabberSubsystem.runGrabber(1);
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
