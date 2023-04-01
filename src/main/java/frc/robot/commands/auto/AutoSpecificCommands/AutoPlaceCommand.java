package frc.robot.commands.auto.AutoSpecificCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.MoveArmDown;
import frc.robot.commands.arm.ReleaseConeSlow;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

public class AutoPlaceCommand extends SequentialCommandGroup {
    public AutoPlaceCommand(ArmSubsystem armSubsystem, GrabberSubsystem grabberSubsystem) {
        addCommands(
            new MoveArmDown(armSubsystem),
            new WaitCommand(0.75),
            new ReleaseConeSlow(grabberSubsystem)
        );
    }
}
