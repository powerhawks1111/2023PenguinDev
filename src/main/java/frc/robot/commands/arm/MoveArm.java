package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class MoveArm extends CommandBase {
    private ArmSubsystem m_armSubsystem;
    private double m_change;
    private boolean isMovingDown = false;
    public MoveArm(ArmSubsystem armSubsystem, double change) {
        m_armSubsystem = armSubsystem;
        m_change = change;
        addRequirements(m_armSubsystem);
    }

    @Override
    public void initialize() {
        isMovingDown = !m_armSubsystem.getStatus();
        m_armSubsystem.setStatus(isMovingDown);
        if (isMovingDown) {
            m_armSubsystem.moveDown(m_change);
        } else {
            m_armSubsystem.moveUp(m_change);
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
