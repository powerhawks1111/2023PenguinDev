package frc.robot.commands.auto.AutoSpecificCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.CombinedCommands.HoldTight;
import frc.robot.commands.intake.*;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeSubSystem;

public class AutoConditional extends SequentialCommandGroup{
    private IntakeSubSystem m_intakeSubsytem;
    private ArmSubsystem m_armSubsystem;
    private GrabberSubsystem m_grabberSubsystem;
    private HoldTight m_holdTight;
    public AutoConditional(IntakeSubSystem intakeSubSystem, ArmSubsystem armSubsystem, GrabberSubsystem grabberSubsystem) {
        m_intakeSubsytem = intakeSubSystem;
        m_armSubsystem = armSubsystem;
        m_grabberSubsystem = grabberSubsystem;
       
        m_holdTight = new HoldTight(m_intakeSubsytem, m_armSubsystem, m_grabberSubsystem);
        addCommands(
            new PositionIntake(m_intakeSubsytem, .275), //.0891
            new IntakeCone(m_intakeSubsytem, m_armSubsystem, m_grabberSubsystem, m_holdTight, false),
            new PositionIntake(m_intakeSubsytem, 2 * Math.PI / 3)
        );
    }
}
