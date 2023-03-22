package frc.robot.commands.CombinedCommands.TeleopCommands;

import javax.sound.midi.Sequence;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.PositionGrabber;
import frc.robot.commands.arm.Score;
import frc.robot.commands.intake.OuttakeCone;
import frc.robot.subsystems.*;

public class ScoreLow extends SequentialCommandGroup{
    private IntakeSubSystem m_intakeSubsytem;
    private ArmSubsystem m_armSubsystem;
    private GrabberSubsystem m_grabberSubsystem;
    public ScoreLow(IntakeSubSystem intakeSubSystem, ArmSubsystem armSubsystem, GrabberSubsystem grabberSubsystem) {
        m_intakeSubsytem = intakeSubSystem;
        m_armSubsystem = armSubsystem;
        m_grabberSubsystem = grabberSubsystem;
      
        addCommands(
            new CloseGrabber(m_grabberSubsystem),
            new OuttakeCone(m_intakeSubsytem),
            new Score(m_armSubsystem, "Low")
        );
}
}
