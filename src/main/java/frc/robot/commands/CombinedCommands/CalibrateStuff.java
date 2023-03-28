package frc.robot.commands.CombinedCommands;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.CalibrateArm;
import frc.robot.commands.intake.CalibrateIntake;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubSystem;

public class CalibrateStuff extends SequentialCommandGroup{
    public IntakeSubSystem m_intakeSubSystem;
    public ArmSubsystem m_armSubsystem;
  

    public CalibrateStuff (IntakeSubSystem intakeSubSystem, ArmSubsystem armSubsystem) {
        m_intakeSubSystem = intakeSubSystem;
        m_armSubsystem = armSubsystem;
       
        //addRequirements(m_intakeSubSystem);
        addCommands(
            new WaitCommand(.1),
            new CalibrateArm(m_armSubsystem),
            new CalibrateIntake(m_intakeSubSystem),
            new PrintCommand("done calibrating")
                   //2.8 radians;
        );
        
    }
}
