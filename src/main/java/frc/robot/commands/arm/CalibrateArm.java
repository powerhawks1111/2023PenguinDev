package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

public class CalibrateArm extends CommandBase{
    private ArmSubsystem m_armSubsystem;
    private Timer m_timer = new Timer();
    
    public CalibrateArm (ArmSubsystem armSubsystem) {
        m_armSubsystem = armSubsystem;
        addRequirements(m_armSubsystem);
    }

    @Override
    public void initialize () {
        m_timer.start();
    }

    @Override 
    public void execute() { 
       
        if (!m_armSubsystem.isCalibrated()) {
            m_armSubsystem.calculateHome();
        }
    }

    @Override
    public boolean isFinished () {
      return true;
    }

    @Override
    public void end (boolean interrupted) {
        
        m_armSubsystem.setCalibrated(true);
    }
}
