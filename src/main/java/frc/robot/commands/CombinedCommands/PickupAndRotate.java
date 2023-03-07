package frc.robot.commands.CombinedCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.CalibrateArm;
import frc.robot.commands.arm.CalibrateGrabber;
import frc.robot.commands.arm.ControlArm;
import frc.robot.commands.arm.PistonArms;
import frc.robot.commands.arm.PositionGrabber;
import frc.robot.commands.intake.CalibrateIntake;
import frc.robot.commands.intake.IntakeCone;
import frc.robot.commands.intake.OuttakeCone;
import frc.robot.commands.intake.PositionIntake;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeSubSystem;

public class PickupAndRotate extends SequentialCommandGroup{
    public IntakeSubSystem m_intakeSubSystem;
    public ArmSubsystem m_armSubsystem;
    public GrabberSubsystem m_grabberSubsystem;

    public PickupAndRotate (IntakeSubSystem intakeSubSystem, ArmSubsystem armSubsystem, GrabberSubsystem grabberSubsystem) {
        m_intakeSubSystem = intakeSubSystem;
        m_armSubsystem = armSubsystem;
        m_grabberSubsystem = grabberSubsystem;
        //addRequirements(m_intakeSubSystem);
        addCommands(
            new PositionIntake(m_intakeSubSystem, -.12),
            //new IntakeCone(m_intakeSubSystem, m_armSubsystem, m_grabberSubsystem),
            new ParallelCommandGroup(
                new PositionIntake(m_intakeSubSystem, 3.075),
                new ControlArm(m_armSubsystem, Math.PI/7)
        
            ),
            new ControlArm(m_armSubsystem, 0),

            new PositionGrabber(m_grabberSubsystem, 18), //18 WORKS FOR FORWARD CONE
            new OuttakeCone(m_intakeSubSystem),
            new ControlArm(m_armSubsystem, Math.PI/1.7)
                 //2.8 radians;
        );
    }
}
