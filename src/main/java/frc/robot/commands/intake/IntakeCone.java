package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.CombinedCommands.HoldTight;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class IntakeCone extends CommandBase{
    private IntakeSubsystem m_intakeSubsystem;
    
    private Timer m_timer = new Timer();
    private HoldTight m_holdTight;
    private boolean m_conditional = false;
    boolean state = false; //this is used to see if we've picked up a cone and stuff

    public IntakeCone(IntakeSubsystem intakeSubSystem, ArmSubsystem armSubsystem, GrabberSubsystem grabberSubsystem, HoldTight holdTight, boolean conditional) {
        m_intakeSubsystem = intakeSubSystem;
        m_holdTight = holdTight; 
        m_conditional = conditional;
        addRequirements(m_intakeSubsystem);
    }

    @Override
    public void initialize() {
        m_timer.start();
    }
    
    @Override
    public void execute() {
        if (!m_intakeSubsystem.hasCone()) {
            m_intakeSubsystem.runIntake(1);
        }
    }

    @Override
    public boolean isFinished() {
        state = ((m_intakeSubsystem.checkAmps()) && m_timer.get() > 0.5) || m_intakeSubsystem.hasCone();
        if (state && m_conditional) {
            m_holdTight.schedule();
        }
        return state;
        //return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_intakeSubsystem.runIntake(0);
        m_timer.stop();
        m_timer.reset();
        m_intakeSubsystem.setCone(state);
    }
    
}
