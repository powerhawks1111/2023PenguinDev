package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class OuttakeCone extends CommandBase{
    private IntakeSubsystem m_intakeSubsystem;
    private Timer m_timer = new Timer();

    public OuttakeCone(IntakeSubsystem intakeSubSystem) {
        m_intakeSubsystem = intakeSubSystem;
        addRequirements(m_intakeSubsystem);
    }

    @Override
    public void initialize() {
        m_timer.start();
    }
    
    @Override
    public void execute() {
        m_intakeSubsystem.runIntake(-.75);
    }

    @Override
    public boolean isFinished() {
        return m_timer.get() >= .35;
        //return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_intakeSubsystem.runIntake(0);
        m_timer.stop();
        m_timer.reset();
        m_intakeSubsystem.setCone(false);
    }
    
}
