package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class MoveArmUp extends CommandBase {
    private ArmSubsystem m_armSubsystem;
    private double m_change;
    public MoveArmUp(ArmSubsystem armSubsystem, double change) {
        m_armSubsystem = armSubsystem;
        m_change = change;
        addRequirements(m_armSubsystem);
    }

    @Override
    public void initialize() {
        m_armSubsystem.moveUp(m_change);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
