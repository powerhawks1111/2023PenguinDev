package frc.robot.commands.CombinedCommands.TeleopCommands;

import javax.sound.midi.Sequence;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.Score;
import frc.robot.commands.intake.OuttakeCone;
import frc.robot.subsystems.*;

public class ScoreHigh extends SequentialCommandGroup{
    private IntakeSubSystem m_intakeSubsytem;
    private ArmSubsystem m_armSubsystem;
    private GrabberSubsystem m_grabberSubsystem;
    public ScoreHigh(IntakeSubSystem intakeSubSystem, ArmSubsystem armSubsystem) {
        m_intakeSubsytem = intakeSubSystem;
        m_armSubsystem = armSubsystem;
      
        addCommands(
           // new OuttakeCone(m_intakeSubsytem),
            new Score(m_armSubsystem, "High")
        );
}
}
