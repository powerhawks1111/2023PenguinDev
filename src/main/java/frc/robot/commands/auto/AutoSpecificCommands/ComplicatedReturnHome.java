package frc.robot.commands.auto.AutoSpecificCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.CombinedCommands.TeleopCommands.*;
import frc.robot.commands.intake.PositionIntake;
import frc.robot.subsystems.*;

public class ComplicatedReturnHome extends SequentialCommandGroup {
    /**
     * Set the intake, arm, and grabber to their home positions
     * @param intakeSubSystem
     * @param armSubsystem
     * @param grabberSubsystem
     */
    public ComplicatedReturnHome (IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem, GrabberSubsystem grabberSubsystem) {
        addCommands(
            new OpenGrabber(grabberSubsystem),
            new WaitCommand(.25), 
            new PositionIntake(intakeSubsystem, 2*Math.PI/3),
            new TeleopReturnHome(armSubsystem, grabberSubsystem)
            // new Open
        );
    }
}
