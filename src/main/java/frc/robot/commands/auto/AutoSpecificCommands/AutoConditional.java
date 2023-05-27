package frc.robot.commands.auto.AutoSpecificCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.CombinedCommands.HoldTight;
import frc.robot.commands.intake.*;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoConditional extends SequentialCommandGroup {
    private HoldTight m_holdTight;
    /**
     * Intake a cone during the autonomous period
     * @param intakeSubsystem
     * @param armSubsystem
     * @param grabberSubsystem
     */
    public AutoConditional(IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem, GrabberSubsystem grabberSubsystem) {
        addCommands(
            new PositionIntake(intakeSubsystem, .41), //.0891 //.275 //.3
            new IntakeCone(intakeSubsystem, armSubsystem, grabberSubsystem, m_holdTight, false),
            new PositionIntake(intakeSubsystem, 2 * Math.PI / 3),
            // added outtakecone (champs pre-playoffs)
            new OuttakeCone(intakeSubsystem),
            new HoldTight(intakeSubsystem, armSubsystem, grabberSubsystem)
        );
    }
}
