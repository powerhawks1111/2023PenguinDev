package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class AutoPlaceCone extends CommandBase {
    private Drivetrain m_drivetrain;
    public AutoPlaceCone(Drivetrain drivetrain) {
        m_drivetrain = drivetrain;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
