package frc.robot.commands.CombinedCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.IntakeCone;
import frc.robot.commands.intake.PositionIntake;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeSubSystem;

public class HumanPlayerStation extends SequentialCommandGroup{
    private IntakeSubSystem m_intakeSubSystem;
    private ArmSubsystem m_armSubsystem;
    private GrabberSubsystem m_grabberSubsystem;
    private HoldTight m_holdTight;
    public HumanPlayerStation(IntakeSubSystem intakeSubSystem, ArmSubsystem armSubsystem, GrabberSubsystem grabberSubsystem, HoldTight holdTight) {
        m_intakeSubSystem = intakeSubSystem;
        m_armSubsystem = armSubsystem;
        m_grabberSubsystem = grabberSubsystem;
        m_holdTight = holdTight;
        addCommands(
            new PositionIntake(m_intakeSubSystem, Math.PI/2.15),
            new IntakeCone(m_intakeSubSystem, m_armSubsystem, m_grabberSubsystem, m_holdTight)
        );
    }
}
 