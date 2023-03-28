package frc.robot.commands.CombinedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.CombinedCommands.TeleopCommands.OpenGrabber;
import frc.robot.commands.arm.ControlArm;
import frc.robot.commands.arm.PositionConditionally;
import frc.robot.commands.arm.PositionGrabber;
import frc.robot.commands.intake.IntakeCone;
import frc.robot.commands.intake.PositionIntake;
import frc.robot.commands.intake.PositionIntakeConditionally;
import frc.robot.commands.intake.SetCone;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeSubSystem;

public class HoldTight extends SequentialCommandGroup {
    private IntakeSubSystem m_intakeSubsytem;
    private ArmSubsystem m_armSubsystem;
    private GrabberSubsystem m_grabberSubsystem;
    public HoldTight(IntakeSubSystem intakeSubSystem, ArmSubsystem armSubsystem, GrabberSubsystem grabberSubsystem) {
        m_intakeSubsytem = intakeSubSystem;
        m_armSubsystem = armSubsystem;
        m_grabberSubsystem = grabberSubsystem;
        
        addCommands(
            new SetCone(m_intakeSubsytem, true),
            

            
            new ParallelCommandGroup(
                new PositionGrabber(m_grabberSubsystem, 3),
                new PositionIntakeConditionally(m_intakeSubsytem),
                new SequentialCommandGroup(new WaitCommand(.25), new ControlArm(m_armSubsystem, Math.PI/8))
            ),
            //!m_armSubsystem.coneDetector() ? new PositionIntake(intakeSubSystem, 3.3) : new WaitCommand(0),
            new PositionConditionally(m_armSubsystem)
            // new WaitCommand(.25)
            //new PositionGrabber(m_grabberSubsystem, 14) //normally 22
        );
    }

    // @Override 
    // public void end (boolean interrupt) {

    // }
}
