package frc.robot.commands.intake;


import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubSystem;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class DefaultPositionIntake extends CommandBase{
    private IntakeSubSystem m_intakeSubsystem;
    
    private Timer m_timer = new Timer();
    private double m_position;
    public DefaultPositionIntake(IntakeSubSystem intakeSubSystem, double position) {
        m_intakeSubsystem = intakeSubSystem;
        m_position = position;
        addRequirements(m_intakeSubsystem);
    }

    @Override
    public void initialize() {
        m_timer.start();
    }
    
    @Override
    public void execute() {
        if (!m_intakeSubsystem.hasCone()) {
            m_intakeSubsystem.positionIntake(m_position);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
        // return m_intakeSubsystem.closeToPosition();
        //return false;
    }

    @Override
    public void end(boolean interrupted) {
        //m_intakeSubsystem.rotateIntake(0);
    }
    
}

