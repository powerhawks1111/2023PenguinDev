package frc.robot.commands.CombinedCommands.TeleopCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.CombinedCommands.TeleopCommands.CloseGrabber;
import frc.robot.commands.CombinedCommands.TeleopCommands.OpenGrabber;
import frc.robot.commands.arm.ReturnHome;
import frc.robot.commands.intake.PositionIntake;
import frc.robot.subsystems.*;

public class TeleopReturnHome extends SequentialCommandGroup{
    private IntakeSubSystem m_intakeSubsystem;
    private ArmSubsystem m_armSubsystem;
    private GrabberSubsystem m_grabberSubsystem;
    public TeleopReturnHome (ArmSubsystem armSubsystem, GrabberSubsystem grabberSubsystem) {
        m_grabberSubsystem = grabberSubsystem;
        m_armSubsystem = armSubsystem;
        addCommands(
            new CloseGrabber(m_grabberSubsystem),
            new ReturnHome(m_armSubsystem),
            new OpenGrabber(m_grabberSubsystem)
            // new Open
        );
    }
}
