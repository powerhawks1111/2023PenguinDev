package frc.robot.commands.auto.AutoSpecificCommands;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ControlArm;
import frc.robot.commands.arm.ReleaseConeFast;
import frc.robot.commands.intake.OuttakeCone;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeSubSystem;

public class AutoScoreLow extends SequentialCommandGroup {
    public AutoScoreLow(ArmSubsystem armSubsystem, GrabberSubsystem grabberSubsystem, IntakeSubSystem intakeSubsystem) {
        addCommands(
            new OuttakeCone(intakeSubsystem),
            new ControlArm(armSubsystem, Math.PI / 2.5),
            new ReleaseConeFast(grabberSubsystem)
        );
    }
}
