package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class MoveArmDown extends CommandBase {
    private ArmSubsystem m_armSubsystem;
    public MoveArmDown(ArmSubsystem armSubsystem) {
        m_armSubsystem = armSubsystem;
        addRequirements(m_armSubsystem);
    }

    @Override
    public void initialize() {
        m_armSubsystem.moveDown();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
