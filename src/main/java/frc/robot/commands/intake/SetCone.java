package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubSystem;

public class SetCone extends CommandBase {
    private IntakeSubSystem m_intakeSubSystem;
    private boolean m_state;
    public SetCone (IntakeSubSystem intakeSubSystem, boolean state) {
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
