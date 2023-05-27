package frc.robot.commands.CombinedCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.CombinedCommands.TeleopCommands.OpenGrabber;
import frc.robot.commands.arm.ControlArm;
import frc.robot.commands.arm.PositionGrabber;
import frc.robot.commands.intake.IntakeCone;
import frc.robot.commands.intake.OuttakeCone;
import frc.robot.commands.intake.PositionIntake;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class EjectCone extends SequentialCommandGroup {
    /**
     * Outtake a cone
     * Alternative method to scoring a cone on the lower level
     * @param intakeSubSystem
     * @param armSubsystem
     * @param grabberSubsystem
     */
    public EjectCone(IntakeSubsystem intakeSubSystem, ArmSubsystem armSubsystem, GrabberSubsystem grabberSubsystem) {
        // m_holdTight = new HoldTight(m_intakeSubsytem, m_armSubsystem, m_grabberSubsystem);
        addCommands(
        
            // new ControlArm(armSubsystem, Math.PI/8),
            new ParallelCommandGroup(
                new PositionIntake(intakeSubSystem, Math.PI/3),
                new SequentialCommandGroup(
                    new WaitCommand(.5),
                    new ControlArm(armSubsystem, 0)
                )
            ),
            new OuttakeCone(intakeSubSystem)
        );
    }
}
