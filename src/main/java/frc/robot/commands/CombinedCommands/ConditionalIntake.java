package frc.robot.commands.CombinedCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.IntakeCone;
import frc.robot.commands.intake.PositionIntake;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeSubSystem;

public class ConditionalIntake extends SequentialCommandGroup{
    private IntakeSubSystem m_intakeSubsytem;
    private ArmSubsystem m_armSubsystem;
    private GrabberSubsystem m_grabberSubsystem;
    private HoldTight m_holdTight;
    public ConditionalIntake(IntakeSubSystem intakeSubSystem, ArmSubsystem armSubsystem, GrabberSubsystem grabberSubsystem) {
        m_intakeSubsytem = intakeSubSystem;
        m_armSubsystem = armSubsystem;
        m_grabberSubsystem = grabberSubsystem;
       
        m_holdTight = new HoldTight(m_intakeSubsytem, m_armSubsystem, m_grabberSubsystem);
        addCommands(
            new PositionIntake(m_intakeSubsytem, .315), //.0891 //.275
            new IntakeCone(m_intakeSubsytem, m_armSubsystem, m_grabberSubsystem, m_holdTight, true)
        );
    }
}
