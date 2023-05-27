package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class PassiveCubeIntake extends CommandBase {
    private GrabberSubsystem m_grabberSubsystem;
    private IntakeSubsystem m_intakeSubsystem;
    private Joystick m_operateJoystick;
    private Joystick m_driveJoystick;
    private Trigger ejectCone;
    private Trigger outtakeCone;

    /**
     * Passive intake that constantly sucks in a cube from the 
     * grabber or intake until is it spit out
     * Both of the systems are run
     * @param grabberSubsystem
     * @param intakeSubsystem
     * @param operateJoystick
     * @param driveJoystick
     */
    public PassiveCubeIntake(GrabberSubsystem grabberSubsystem, IntakeSubsystem intakeSubsystem, Joystick operateJoystick, Joystick driveJoystick) {
        m_grabberSubsystem = grabberSubsystem;
        m_intakeSubsystem = intakeSubsystem;
        m_operateJoystick = operateJoystick;   
        m_driveJoystick = driveJoystick;
    }

    @Override
    public void initialize () {
        // eject through grabber
        ejectCone = new JoystickButton(m_operateJoystick, 6);
        // outtake through intake
        outtakeCone = new JoystickButton(m_driveJoystick, 5);
    }

    @Override 
    public void execute() {
        // run grabber
        m_grabberSubsystem.runGrabber(.1); //.05 sucks for high
        // run intake
        m_intakeSubsystem.runIntake(.2); // first testing value is .4
    }

    @Override
    public boolean isFinished() {
        return ejectCone.getAsBoolean() || outtakeCone.getAsBoolean();
    }

    @Override
    public void end(boolean interrupt) {
        m_grabberSubsystem.runGrabber(0);
        m_intakeSubsystem.runIntake(0);
        m_intakeSubsystem.setCone(false);
    }
}
