package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ReturnHome extends CommandBase{
    private ArmSubsystem m_armSubsystem;
    
    public ReturnHome(ArmSubsystem armSubsystem) {
        m_armSubsystem = armSubsystem;
        addRequirements(m_armSubsystem);
    }
    @Override
    public void initialize() {

    }

    @Override 
    public void execute() {
        m_armSubsystem.positionArm(0); 
        m_armSubsystem.deployPistons(false);
    }

    @Override 
    public boolean isFinished () {
        return m_armSubsystem.closeToPosition();
    }

    

}
