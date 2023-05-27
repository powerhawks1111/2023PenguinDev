package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class CalibrateIntake extends CommandBase{
    private IntakeSubsystem m_intakeSubsystem;
    private Timer m_timer = new Timer();
    public CalibrateIntake (IntakeSubsystem intakeSubsystem) {
        m_intakeSubsystem = intakeSubsystem;
        addRequirements(m_intakeSubsystem);
    }

    @Override
    public void initialize () {
        m_timer.start();
    }

    @Override 
    public void execute() { 
        if (!m_intakeSubsystem.isCalibrated()) {
            m_intakeSubsystem.calculateIntakeHome();
        }
    }

    @Override
    public boolean isFinished () {
      return true;
    }

    @Override
    public void end (boolean interrupted) {
        
        
        m_intakeSubsystem.setCalibrated(true);
    }
}
