package frc.robot.commands.CombinedCommands.TeleopCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ReleaseConeSlow;
import frc.robot.commands.arm.Score;
import frc.robot.commands.intake.OuttakeCone;
import frc.robot.subsystems.*;

public class ScoreMid extends SequentialCommandGroup {
    /**
     * Score a cone in the middle position
     */
    private IntakeSubsystem m_intakeSubsytem;
    private ArmSubsystem m_armSubsystem;
    private GrabberSubsystem m_grabberSubsystem;
    public ScoreMid(IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem, GrabberSubsystem grabberSubsystem) {      
        addCommands(
            // new CloseGrabber(grabberSubsystem),
            new OuttakeCone(intakeSubsystem),
            new Score(armSubsystem, "Mid")
        );
    }
}
