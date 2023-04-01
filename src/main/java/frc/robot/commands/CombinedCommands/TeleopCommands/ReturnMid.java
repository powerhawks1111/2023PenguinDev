package frc.robot.commands.CombinedCommands.TeleopCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ArmSubsystem;

public class ReturnMid extends CommandBase{
    private ArmSubsystem m_armSubsystem;
    private boolean m_conditional;
    private Timer m_timer = new Timer();
    /**
     * Position the arm to its set home position and retract the arm pistons
     * @param armSubsystem
     */
    public ReturnMid(ArmSubsystem armSubsystem) {
        m_armSubsystem = armSubsystem;
        addRequirements(m_armSubsystem);
    }
    @Override
    public void initialize() {
        m_timer.start();
        m_conditional = Math.abs(m_armSubsystem.getArmPosition()) > 3 * Math.PI / 4;
    }

    @Override 
    public void execute() {
        m_armSubsystem.deployPistons(false);
        if (m_conditional && m_timer.get() > .5) {
            m_armSubsystem.positionArm(Math.PI / 2); 
        } else if (!m_conditional) {
            m_armSubsystem.positionArm(Math.PI / 2);
        }
    }

    @Override 
    public boolean isFinished () {
        if (!m_conditional){
            return m_armSubsystem.closeToPosition();
        } else {
            if (m_timer.get() > .5) {
                return m_armSubsystem.closeToPosition();
            } else {
                return false;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
        m_timer.reset();
    }
}
