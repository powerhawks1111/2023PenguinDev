package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

public class CalibrateArm extends CommandBase{
    private ArmSubsystem m_armSubsystem;
    public CalibrateArm (ArmSubsystem armSubsystem) {
        m_armSubsystem = armSubsystem;
        addRequirements(m_armSubsystem);
    }

    @Override
    public void initialize () {

    }

    @Override 
    public void execute() { 
        if (!m_armSubsystem.isCalibrated()) {
            m_armSubsystem.calculateHome();
        }
        
    }

    @Override
    public boolean isFinished () {
      return m_armSubsystem.isCalibrated();
    }

    @Override
    public void end (boolean interrupted) {
    
    }
}
