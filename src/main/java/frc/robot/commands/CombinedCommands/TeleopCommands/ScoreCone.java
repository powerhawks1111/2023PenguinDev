package frc.robot.commands.CombinedCommands.TeleopCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ReleaseConeFast;
import frc.robot.commands.arm.ReleaseConeSlow;
import frc.robot.commands.arm.SpinGrabber;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

public class ScoreCone extends SequentialCommandGroup {
    public ScoreCone(GrabberSubsystem grabberSubsystem, ArmSubsystem armSubsystem) {
        addCommands(
            new ReleaseConeSlow(grabberSubsystem),
            new ReturnMid(armSubsystem)
        );
    }
}
