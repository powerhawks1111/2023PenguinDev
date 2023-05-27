package frc.robot.commands.CombinedCommands.TeleopCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ReleaseConeSlow;
import frc.robot.commands.arm.Score;
import frc.robot.commands.intake.OuttakeCone;
import frc.robot.subsystems.*;

public class ScoreHigh extends SequentialCommandGroup {
    /**
     * Score a cone in the high position
     * @param intakeSubsytem
     * @param armSubsystem
     * @param grabberSubsystem
     */
    public ScoreHigh(IntakeSubsystem intakeSubsytem, ArmSubsystem armSubsystem, GrabberSubsystem grabberSubsystem) {      
        addCommands(
            // new CloseGrabber(grabberSubsystem),
            new OuttakeCone(intakeSubsytem),
            new Score(armSubsystem, "High")
        );
    }
}
