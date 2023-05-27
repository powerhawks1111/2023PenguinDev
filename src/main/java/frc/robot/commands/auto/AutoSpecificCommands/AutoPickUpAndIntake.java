package frc.robot.commands.auto.AutoSpecificCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.CombinedCommands.HoldTight;
import frc.robot.commands.auto.HoldCommand;
import frc.robot.commands.intake.IntakeCone;
import frc.robot.commands.intake.OuttakeCone;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

public class AutoPickUpAndIntake extends SequentialCommandGroup {
    public AutoPickUpAndIntake(IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem, GrabberSubsystem grabberSubsystem) {
        addCommands(
            new OuttakeCone(intakeSubsystem),
            new HoldTight(intakeSubsystem, armSubsystem, grabberSubsystem)
        );
    }
}
