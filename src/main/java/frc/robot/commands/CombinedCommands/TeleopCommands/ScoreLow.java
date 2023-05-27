package frc.robot.commands.CombinedCommands.TeleopCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.Score;
import frc.robot.commands.intake.OuttakeCone;
import frc.robot.commands.arm.ReleaseConeSlow;
import frc.robot.subsystems.*;

public class ScoreLow extends SequentialCommandGroup {
    /**
     * Score a cone in the low position
     * @param intakeSubsystem
     * @param armSubsystem
     * @param grabberSubsystem
     */
    public ScoreLow(IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem, GrabberSubsystem grabberSubsystem) { 
        addCommands(
            // new CloseGrabber(grabberSubsystem),
            new OuttakeCone(intakeSubsystem),
            new Score(armSubsystem, "Low")
        );
}
}
