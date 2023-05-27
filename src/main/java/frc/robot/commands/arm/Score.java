package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

public class Score extends CommandBase{ 
    private ArmSubsystem m_armSubsystem;
    private String m_height;

    /**
     * Score the cone at the specified position
     * @param armSubsystem
     * @param height low, mid, or high
     */
    public Score(ArmSubsystem armSubsystem, String height) {
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
            m_armSubsystem.positionArm(.589); //pi/8
            m_armSubsystem.deployPistons(false);
        }
        if (m_height == "Mid") {
            m_armSubsystem.positionArm(1.524); // 1.409
            m_armSubsystem.deployPistons(false);
        }
        if (m_height == "High") {
            m_armSubsystem.positionArm(3.14);
            // m_armSubsystem.deployPistons(true);
            if (m_armSubsystem.getArmPosition() > Math.PI / 4.5) {
                m_armSubsystem.deployPistons(true);
            }
            // if (m_armSubsystem.getArmPosition() > Math.PI / 2) {
            //     m_armSubsystem.deployPistons(true);
            // }
        }
        if (m_height == "AutoCube") {
            m_armSubsystem.positionArm(3.02); //3.28 (was too high)
            if (m_armSubsystem.getArmPosition() > Math.PI / 2) {
                m_armSubsystem.deployPistons(true);
            }
            // m_armSubsystem.deployPistons(false);
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
