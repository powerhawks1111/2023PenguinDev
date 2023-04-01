package frc.robot.commands.CombinedCommands.TeleopCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.CombinedCommands.TeleopCommands.CloseGrabber;
import frc.robot.commands.CombinedCommands.TeleopCommands.OpenGrabber;
import frc.robot.commands.arm.ReturnHome;
import frc.robot.subsystems.*;

public class TeleopReturnHome extends SequentialCommandGroup {
    /**
     * Move the arm to its home position and end with the grabber open
     * @param armSubsystem
     * @param grabberSubsystem
     */
    public TeleopReturnHome (ArmSubsystem armSubsystem, GrabberSubsystem grabberSubsystem) {
        addCommands(
            // new CloseGrabber(grabberSubsystem),
            new ReturnHome(armSubsystem)
            // new OpenGrabber(grabberSubsystem)
            // new Open
        );
    }
}
