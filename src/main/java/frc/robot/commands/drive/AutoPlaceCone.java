package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.MoveArmDown;
import frc.robot.commands.arm.ReleaseConeSlow;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

public class AutoPlaceCone extends SequentialCommandGroup {
    public AutoPlaceCone(ArmSubsystem armSubsystem, GrabberSubsystem grabberSubsystem) {
        addCommands(
            new MoveArmDown(armSubsystem, 5),
            new ReleaseConeSlow(grabberSubsystem)
        );
    }
}
