package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Drivetrain;
import frc.robot.variables.Objects;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Timer;

public class BalanceCommand extends CommandBase { 
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private Drivetrain m_drivetrain;
    // private double timerNow = 0;
    // private double timerPrevious = 0;
    // private double anglePreviousRoll = 0;
    // private double anglePreviousPitch = 0;
    /**
     * Command for balancing the robot on the charging station
     * Uses navx gyro outputs
     * @param subsystem The subsystem used by this command.
     */
    public BalanceCommand(Drivetrain subsystem) {
      m_drivetrain = subsystem;
      addRequirements(m_drivetrain);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      m_drivetrain.balanceRobot();
      System.out.println("balancing");
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      m_drivetrain.drive(0, 0, 0, false, true);
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return m_drivetrain.isBalanced();
    }
}
