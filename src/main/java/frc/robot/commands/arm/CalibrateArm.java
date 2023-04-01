package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class CalibrateArm extends CommandBase{
    private ArmSubsystem m_armSubsystem;
    /**
     * Calibrate the arm and get the home position
     * @param armSubsystem
     */
    public CalibrateArm (ArmSubsystem armSubsystem) {
        m_armSubsystem = armSubsystem;
        addRequirements(m_armSubsystem);
    }

    @Override
    public void initialize () {}

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
