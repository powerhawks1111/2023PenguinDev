package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class PistonArms extends CommandBase {
    private ArmSubsystem m_armSubsystem;
    private boolean m_deploy;
    
    public PistonArms(ArmSubsystem armSubsystem, boolean deploy) {
        m_armSubsystem = armSubsystem;
        m_deploy = deploy;
        addRequirements(m_armSubsystem);

    }

    @Override
    public void initialize() {
        if (!m_deploy)
            m_armSubsystem.deployPistons(m_deploy);
    }
    @Override 
    public boolean isFinished() {
        return false;
    }

    @Override 
    public void end(boolean interrupt) {
       // m_armSubsystem.deployPistons(!m_deploy);
    }
}
