package frc.robot.commands.auto.AutoSpecificCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.CombinedCommands.TeleopCommands.ScoreCube;
import frc.robot.commands.arm.ReleaseConeFast;
import frc.robot.commands.arm.SpinGrabberTimed;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoCubeHigh extends SequentialCommandGroup {
    public AutoCubeHigh(GrabberSubsystem grabberSubsystem, IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem) {
        addCommands(
            new SpinGrabberTimed(grabberSubsystem, 1),
            new ParallelCommandGroup(
                // might have to change this height (below)
                new ScoreCube(intakeSubsystem, armSubsystem, grabberSubsystem),
                new SequentialCommandGroup(
                    new WaitCommand(.05),
                    new SpinGrabberTimed(grabberSubsystem, 1),
                    new WaitCommand(.05),
                    new SpinGrabberTimed(grabberSubsystem, 1)
                    // repeat this twice if it doesn't work
                )
            ),
            new WaitCommand(.2),
            new ReleaseConeFast(grabberSubsystem)
        );
    }
}
