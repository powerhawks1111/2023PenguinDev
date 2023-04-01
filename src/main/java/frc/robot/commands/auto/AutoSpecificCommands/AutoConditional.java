package frc.robot.commands.auto.AutoSpecificCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.CombinedCommands.HoldTight;
import frc.robot.commands.intake.*;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeSubSystem;

public class AutoConditional extends SequentialCommandGroup {
    private HoldTight m_holdTight;
    /**
     * Intake a cone during the autonomous period
     * @param intakeSubsystem
     * @param armSubsystem
     * @param grabberSubsystem
     */
    public AutoConditional(IntakeSubSystem intakeSubsystem, ArmSubsystem armSubsystem, GrabberSubsystem grabberSubsystem) {
        addCommands(
            new PositionIntake(intakeSubsystem, .275), //.0891
            new IntakeCone(intakeSubsystem, armSubsystem, grabberSubsystem, m_holdTight, false),
            new PositionIntake(intakeSubsystem, 2 * Math.PI / 3),
            new HoldTight(intakeSubsystem, armSubsystem, grabberSubsystem)
        );
    }
}
