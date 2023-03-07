package frc.robot.commands.arm;

import java.util.ResourceBundle.Control;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ControlArm extends CommandBase {
    private ArmSubsystem m_armSubsystem;
    private double m_position;

    public ControlArm(ArmSubsystem subsytem, double position) {
        m_armSubsystem = subsytem;
        m_position = position;
        addRequirements(m_armSubsystem);
    }

    @Override
    public void execute() {
     //m_armSubsystem.runArmMotors(m_speed);
        m_armSubsystem.positionArm(m_position);
        //m_armSubsystem.deployPistons(true);
    }

    @Override
    public boolean isFinished() {
        return m_armSubsystem.closeToPosition();
    }

    @Override
    public void end(boolean interrupted) {
        //m_armSubsystem.deployPistons(false);
    //    m_armSubsystem.runArmMotors(0);
    }



}
