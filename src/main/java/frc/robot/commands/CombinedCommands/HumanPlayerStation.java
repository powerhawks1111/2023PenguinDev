package frc.robot.commands.CombinedCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.ControlArm;
import frc.robot.commands.arm.PositionConditionally;
import frc.robot.commands.arm.SpinGrabber;
import frc.robot.commands.intake.IntakeCone;
import frc.robot.commands.intake.PositionIntake;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class HumanPlayerStation extends SequentialCommandGroup{
    private HoldTight m_holdTight;
    /**
     * Position the intake to pick up cones from the human player station (NOT WORKING)
     * @param intakeSubsystem
     * @param armSubsystem
     * @param grabberSubsystem
     * @param holdTight
     */
    public HumanPlayerStation(IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem, GrabberSubsystem grabberSubsystem, HoldTight holdTight) {
        m_holdTight = holdTight;
        addCommands(
            // new PositionIntake(intakeSubSystem, Math.PI/2.1), // was 2.15
            // new IntakeCone(intakeSubSystem, armSubsystem, grabberSubsystem, m_holdTight, true)
            new PositionIntake(intakeSubsystem, Math.PI),
            new WaitCommand(.5),
            new ControlArm(armSubsystem, -1.50), //-1.54
            new SpinGrabber(grabberSubsystem, 1)
        );
    }
}
 