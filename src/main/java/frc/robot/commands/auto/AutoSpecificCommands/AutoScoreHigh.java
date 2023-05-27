package frc.robot.commands.auto.AutoSpecificCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.CombinedCommands.PlaceCommand;
import frc.robot.commands.drive.AutoPlaceCone;
import frc.robot.commands.intake.OuttakeCone;
import frc.robot.commands.intake.PositionIntake;
import frc.robot.commands.intake.SetCone;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoScoreHigh extends SequentialCommandGroup {
    public AutoScoreHigh(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, GrabberSubsystem grabberSubsystem) {
        addCommands(
            new SetCone(intakeSubsystem, false),
            new PositionIntake(intakeSubsystem, 3.3),
            new ReverseScoreHigh(armSubsystem),
            new AutoPlaceCone(armSubsystem, grabberSubsystem)
        );
    }
}
