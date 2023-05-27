package frc.robot.commands.CombinedCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.ControlArm;
import frc.robot.commands.arm.MoveArm;
import frc.robot.commands.arm.ReturnHome;
import frc.robot.commands.intake.PositionIntake;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class HumanPlayerReturnHome extends SequentialCommandGroup {
    public HumanPlayerReturnHome(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem) {
        addCommands(
            new PositionIntake(intakeSubsystem, Math.PI),
            new WaitCommand(1),
            // m_armSubsystem.positionArm(Math.PI/8);
            new ControlArm(armSubsystem, Math.PI/8)
            // new ReturnHome(armSubsystem)
            
        );
    }
}
