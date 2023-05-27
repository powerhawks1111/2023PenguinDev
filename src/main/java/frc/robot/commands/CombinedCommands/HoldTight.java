package frc.robot.commands.CombinedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.CombinedCommands.TeleopCommands.OpenGrabber;
import frc.robot.commands.arm.ControlArm;
import frc.robot.commands.arm.PositionConditionally;
import frc.robot.commands.arm.PositionGrabber;
import frc.robot.commands.arm.SpinGrabberTimed;
import frc.robot.commands.intake.IntakeCone;
import frc.robot.commands.intake.OuttakeCone;
import frc.robot.commands.intake.PositionIntake;
import frc.robot.commands.intake.PositionIntakeConditionally;
import frc.robot.commands.intake.SetCone;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class HoldTight extends SequentialCommandGroup {
    /**
     * After the cone is spin out from the intake, the arm can
     * take control of the cone to be placed at a later time
     * @param intakeSubsystem
     * @param armSubsystem
     * @param grabberSubsystem
     */
    public HoldTight(IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem, GrabberSubsystem grabberSubsystem) {       
        addCommands(
            new SetCone(intakeSubsystem, true),
            new ParallelCommandGroup(
                // new PositionGrabber(grabberSubsystem, 3),
                new PositionIntakeConditionally(intakeSubsystem),
                new SequentialCommandGroup(
                    new ControlArm(armSubsystem, Math.PI/8)
                )
            ),
            //!m_armSubsystem.coneDetector() ? new PositionIntake(intakeSubSystem, 3.3) : new WaitCommand(0),
            new WaitCommand(.25),
            new PositionConditionally(armSubsystem),
            new WaitCommand(.2),
            new PositionIntake(intakeSubsystem, 2.9),  
            new SpinGrabberTimed(grabberSubsystem, 1)
            // new OuttakeCone(intakeSubsystem)
            // new WaitCommand(.25)
            //new PositionGrabber(m_grabberSubsystem, 14) //normally 22
        );
    }
}
