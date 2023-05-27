package frc.robot.commands.CombinedCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.MoveArmDown;
import frc.robot.commands.arm.ReleaseConeSlow;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

public class PlaceCommand extends SequentialCommandGroup {
    public PlaceCommand(ArmSubsystem armSubsystem, GrabberSubsystem grabberSubsystem) {
        addCommands(
            new MoveArmDown(armSubsystem, 5)
            // new WaitCommand(0.75),
            // new ReleaseConeSlow(grabberSubsystem)
        );
    }
}
