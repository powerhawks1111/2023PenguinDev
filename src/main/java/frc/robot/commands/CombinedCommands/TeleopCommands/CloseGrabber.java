package frc.robot.commands.CombinedCommands.TeleopCommands;

import javax.sound.midi.Sequence;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.PositionGrabber;
import frc.robot.commands.arm.Score;
import frc.robot.commands.intake.OuttakeCone;
import frc.robot.subsystems.*;

public class CloseGrabber extends SequentialCommandGroup{
    private IntakeSubSystem m_intakeSubsytem;
    private ArmSubsystem m_armSubsystem;
    private GrabberSubsystem m_grabberSubsystem;
    public CloseGrabber(GrabberSubsystem grabberSubsystem) {
        m_grabberSubsystem = grabberSubsystem;
      
        addCommands(
            new PositionGrabber(grabberSubsystem, 28)
        );
}
}
