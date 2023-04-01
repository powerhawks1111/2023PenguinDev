package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.TrajectoryFollower;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Drivetrain;

public class simpleTrajectory extends SequentialCommandGroup {
    /*
     * THIS FILE IS NOT BEING USED 
     */
    public simpleTrajectory (Drivetrain drive) {
        addCommands(new TrajectoryFollower(drive));
    }
}
