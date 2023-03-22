package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

public class Score extends CommandBase{ 
    private ArmSubsystem m_armSubsystem;
    private GrabberSubsystem m_grabberSubsystem;
    private Joystick m_driverJoystick;
    private String m_height;


    public Score (ArmSubsystem armSubsystem, String height) {
        m_armSubsystem = armSubsystem;
        
        m_height = height;
        

        addRequirements(m_armSubsystem);
    }
    @Override
    public void initialize () {
        m_armSubsystem.positionArm(Math.PI/6);
        m_armSubsystem.setScoringButDangerous(true);
    }

    @Override 
    public void execute() {
        if (m_height == "Low") {
            m_armSubsystem.positionArm(Math.PI/8);
        }
        if (m_height == "Mid") {
            m_armSubsystem.positionArm(Math.PI/2-.1);
        }
        if (m_height == "High") {
            m_armSubsystem.positionArm(3.1);
            m_armSubsystem.deployPistons(true);
        }
    }

    @Override
    public boolean isFinished() {
        return m_armSubsystem.closeToPosition();
    }

    @Override
    public void end(boolean interrupt){
        // m_grabberSubsystem.positionGrabber(0);
        // m_armSubsystem.deployPistons(true);
        // m_armSubsystem.positionArm(0);
    }
    
}
