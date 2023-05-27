package frc.robot.commands.intake;

import java.util.ResourceBundle.Control;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class PositionIntakeConditionally extends CommandBase {
    private IntakeSubsystem m_intakeSubsystem;
    private double m_position;

    public PositionIntakeConditionally(IntakeSubsystem subsytem) {
        m_intakeSubsystem = subsytem;
        // m_position = position;
        addRequirements(m_intakeSubsystem);
    }

    @Override
    public void execute() {
     //m_armSubsystem.runArmMotors(m_speed);
        m_intakeSubsystem.positionConditionally();
        //m_armSubsystem.deployPistons(true);
    }

    @Override
    public boolean isFinished() {
        return m_intakeSubsystem.closeToPosition();
    }

    @Override
    public void end(boolean interrupted) {
        //m_armSubsystem.deployPistons(false);
    //    m_armSubsystem.runArmMotors(0);
    }



}
