package frc.robot.commands.CombinedCommands.TeleopCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.intake.OuttakeCone;
import frc.robot.commands.intake.PositionIntake;
import frc.robot.subsystems.IntakeSubsystem;

public class OuttakeBackwards extends SequentialCommandGroup {
    /*
     * THIS FILE IS NOT BEING USED 
     */
    /**
     * Outtake a cone
     * Alternative method to scoring a cone on the lower level
     * @param intakeSubSystem
     */
    public OuttakeBackwards(IntakeSubsystem intakeSubSystem) {
        addCommands(
            new PositionIntake(intakeSubSystem, 3.15),
            new WaitCommand(.5),
            new OuttakeCone(intakeSubSystem)
        );
    }
}
