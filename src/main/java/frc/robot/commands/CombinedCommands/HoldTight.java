package frc.robot.commands.CombinedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.ControlArm;
import frc.robot.commands.arm.PositionGrabber;
import frc.robot.commands.intake.IntakeCone;
import frc.robot.commands.intake.PositionIntake;
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
            new ParallelCommandGroup(
                new PositionIntake(m_intakeSubsytem, 2.95),
                new ControlArm(m_armSubsystem, Math.PI/8)
            ),
            new ControlArm(m_armSubsystem, 0),
            new WaitCommand(.25),
            new PositionGrabber(m_grabberSubsystem, 22)
        );
    }

    // @Override 
    // public void end (boolean interrupt) {

    // }
}
