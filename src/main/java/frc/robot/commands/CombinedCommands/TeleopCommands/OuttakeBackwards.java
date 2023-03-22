package frc.robot.commands.CombinedCommands.TeleopCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.OuttakeCone;
import frc.robot.commands.intake.PositionIntake;
import frc.robot.subsystems.IntakeSubSystem;

public class OuttakeBackwards extends SequentialCommandGroup {
    private IntakeSubSystem m_intakeSubSystem;
    public OuttakeBackwards (IntakeSubSystem intakeSubSystem) {
        m_intakeSubSystem = intakeSubSystem;
        addCommands(
            new PositionIntake(m_intakeSubSystem, 3.15),
            new OuttakeCone(m_intakeSubSystem)
        );
    }
}
