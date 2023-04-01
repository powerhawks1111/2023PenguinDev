package frc.robot.commands.CombinedCommands.TeleopCommands;

import javax.sound.midi.Sequence;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.PositionGrabber;
import frc.robot.commands.arm.Score;
import frc.robot.commands.intake.OuttakeCone;
import frc.robot.subsystems.*;

public class OpenGrabber extends SequentialCommandGroup {
    /**
     * Move the grabber to its open position
     * @param grabberSubsystem
     */
    public OpenGrabber(GrabberSubsystem grabberSubsystem) {      
        addCommands(
            new PositionGrabber(grabberSubsystem, 0)
        );
    }
}
