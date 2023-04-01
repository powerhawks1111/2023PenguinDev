package frc.robot.commands.CombinedCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.IntakeCone;
import frc.robot.commands.intake.PositionIntake;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeSubSystem;

public class ConditionalIntake extends SequentialCommandGroup{
    private HoldTight m_holdTight;
    /**
     * Lower the intake and spin the intake wheels
     * @param intakeSubsystem
     * @param armSubsystem
     * @param grabberSubsystem
     */
    public ConditionalIntake(IntakeSubSystem intakeSubsystem, ArmSubsystem armSubsystem, GrabberSubsystem grabberSubsystem) {
        m_holdTight = new HoldTight(intakeSubsystem, armSubsystem, grabberSubsystem);
        addCommands(
            new PositionIntake(intakeSubsystem, .3), //.0891 //.275
            new IntakeCone(intakeSubsystem, armSubsystem, grabberSubsystem, m_holdTight, false),
            new PositionIntake(intakeSubsystem, 2 * Math.PI / 3)
        );
    }
}
