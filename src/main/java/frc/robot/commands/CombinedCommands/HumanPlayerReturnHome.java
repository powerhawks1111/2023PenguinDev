package frc.robot.commands.CombinedCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.ReturnHome;
import frc.robot.commands.intake.PositionIntake;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubSystem;

public class HumanPlayerReturnHome extends SequentialCommandGroup {
    public HumanPlayerReturnHome(ArmSubsystem armSubsystem, IntakeSubSystem intakeSubsystem) {
        addCommands(
            new PositionIntake(intakeSubsystem, Math.PI),
            new WaitCommand(.5),
            new ReturnHome(armSubsystem)
        );
    }
}
