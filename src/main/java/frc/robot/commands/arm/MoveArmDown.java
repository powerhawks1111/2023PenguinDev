package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class MoveArmDown extends CommandBase {
    private ArmSubsystem m_armSubsystem;
    private double m_change;
    public MoveArmDown(ArmSubsystem armSubsystem, double change) {
        m_armSubsystem = armSubsystem;
        m_change = change;
        addRequirements(m_armSubsystem);
    }

    @Override
    public void initialize() {
        m_armSubsystem.moveDown(m_change);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
