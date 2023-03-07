package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeSubSystem;

public class CalibrateIntake extends CommandBase{
    private IntakeSubSystem m_intakeSubsystem;
    public CalibrateIntake (IntakeSubSystem intakeSubsystem) {
        m_intakeSubsystem = intakeSubsystem;
        addRequirements(m_intakeSubsystem);
    }

    @Override
    public void initialize () {

    }

    @Override 
    public void execute() { 
        if (!m_intakeSubsystem.isCalibrated()) {
            m_intakeSubsystem.calculateIntakeHome();
        }
        
    }

    @Override
    public boolean isFinished () {
      return m_intakeSubsystem.isCalibrated();
    }

    @Override
    public void end (boolean interrupted) {
    
    }
}
