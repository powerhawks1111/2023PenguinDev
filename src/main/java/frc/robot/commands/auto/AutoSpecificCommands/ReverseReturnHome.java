package frc.robot.commands.auto.AutoSpecificCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.PositionConditionally;
import frc.robot.commands.arm.ReturnHome;
import frc.robot.commands.intake.PositionIntake;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ReverseReturnHome extends SequentialCommandGroup {
    public ReverseReturnHome(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, GrabberSubsystem grabberSubsystem) {
        addCommands(
            new ParallelCommandGroup(
                new PositionIntake(intakeSubsystem, .3),
                // new AutoConditional(intakeSubsystem, armSubsystem, grabberSubsystem),
                new SequentialCommandGroup(
                    new AutoReturnHome(armSubsystem)
                )
            )
        );
    }
}
