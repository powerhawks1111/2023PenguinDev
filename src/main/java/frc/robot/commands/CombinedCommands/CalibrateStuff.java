package frc.robot.commands.CombinedCommands;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.CalibrateArm;
import frc.robot.commands.intake.CalibrateIntake;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class CalibrateStuff extends SequentialCommandGroup {
    /**
     * Calibrate the arm and intake for the autonomous period
     * @param intakeSubSystem
     * @param armSubsystem
     */
    public CalibrateStuff(IntakeSubsystem intakeSubSystem, ArmSubsystem armSubsystem) {       
        addCommands(
            new WaitCommand(.1),
            new CalibrateArm(armSubsystem),
            new CalibrateIntake(intakeSubSystem),
            new PrintCommand("done calibrating")
                   //2.8 radians;
        );
    }
}
