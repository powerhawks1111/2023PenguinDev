package frc.robot.commands.CombinedCommands.TeleopCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.PositionGrabber;
import frc.robot.subsystems.*;

public class CloseGrabber extends SequentialCommandGroup {
    /**
     * Move the grabber to its closed position
     * @param grabberSubsystem
     */
    public CloseGrabber(GrabberSubsystem grabberSubsystem) {      
        addCommands(
            new PositionGrabber(grabberSubsystem, 28)
        );
    }
}
