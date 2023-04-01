package frc.robot.commands.auto.AutoSpecificCommands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ReverseScoreHigh extends CommandBase {
    private ArmSubsystem m_armSubsystem;
    public ReverseScoreHigh(ArmSubsystem armSubsystem) {
        m_armSubsystem = armSubsystem;
        addRequirements(m_armSubsystem);
    }

    @Override
    public void execute() {
        m_armSubsystem.positionArm(-Math.PI);
        m_armSubsystem.deployPistons(true);
    }

    @Override
    public boolean isFinished() {
        return m_armSubsystem.closeToPosition();
    }

    @Override
    public void end(boolean interrupted) {

    }
}
