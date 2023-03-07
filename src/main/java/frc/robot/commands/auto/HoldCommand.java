package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Drivetrain;

public class HoldCommand extends CommandBase{
    private Drivetrain m_drivetrain;

    public HoldCommand (Drivetrain drivetrain) {
        m_drivetrain = drivetrain;
        addRequirements(m_drivetrain);
    }

    @Override
    public void execute () {
        m_drivetrain.drive(0, 0, 0, false, true);;
    }
    

    @Override
    public boolean isFinished () {
        return false;
    }
}

