package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class SetCone extends CommandBase {
    private IntakeSubsystem m_intakeSubSystem;
    private boolean m_state;
    /**
     * 
     * @param intakeSubSystem
     * @param state
     */
    public SetCone (IntakeSubsystem intakeSubSystem, boolean state) {
        m_intakeSubSystem = intakeSubSystem;
        m_state = state;
        addRequirements(m_intakeSubSystem);
    }
    
    @Override
    public void initialize() {
        m_intakeSubSystem.setCone(m_state);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
