package frc.robot.commands.CombinedCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.ControlArm;
import frc.robot.commands.intake.IntakeCone;
import frc.robot.commands.intake.OuttakeCone;
import frc.robot.commands.intake.PositionIntake;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeSubSystem;

public class EjectCone extends SequentialCommandGroup{
    private IntakeSubSystem m_intakeSubsytem;
    private ArmSubsystem m_armSubsystem;
    private GrabberSubsystem m_grabberSubsystem;
    private HoldTight m_holdTight;
    public EjectCone(IntakeSubSystem intakeSubSystem, ArmSubsystem armSubsystem) {
        m_intakeSubsytem = intakeSubSystem;
        m_armSubsystem = armSubsystem;
        // m_armSubsystem = armSubsystem;
        // m_grabberSubsystem = grabberSubsystem;
       
        // m_holdTight = new HoldTight(m_intakeSubsytem, m_armSubsystem, m_grabberSubsystem);
        addCommands(
            new ControlArm(m_armSubsystem, Math.PI/8),
            new ParallelCommandGroup(
                new PositionIntake(m_intakeSubsytem, Math.PI/3),
                new SequentialCommandGroup(
                    new WaitCommand(.5),
                    new ControlArm(m_armSubsystem, 0)
                )
            ),
             
            new OuttakeCone(m_intakeSubsytem)
        );
    }
}
