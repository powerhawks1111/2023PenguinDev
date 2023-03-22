package frc.robot.commands.drive;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeSubSystem;

public class OperatorOverride extends CommandBase {
    private Joystick operateJoystick;
    private ArmSubsystem m_armSubsystem;
    private IntakeSubSystem m_intakeSubSystem;
    private GrabberSubsystem m_grabberSubsystem;

    
    public OperatorOverride(ArmSubsystem armSubsystem, IntakeSubSystem intakeSubSystem, GrabberSubsystem grabberSubsystem, Joystick operatorJoystick) {
        m_armSubsystem = armSubsystem;
        m_intakeSubSystem = intakeSubSystem;
        m_grabberSubsystem = grabberSubsystem;
        operateJoystick = operatorJoystick;
        addRequirements(m_armSubsystem);
        addRequirements(m_intakeSubSystem);
        addRequirements(m_grabberSubsystem); //TODO ADD BRAKE MODE COMMANDS

    }   

    @Override
    public void initialize () {
        CommandScheduler.getInstance().cancelAll();
        m_armSubsystem.brakeMode();
        m_intakeSubSystem.brakeMode();

    }

    @Override
    public void execute() {
        double intakeMovement = operateJoystick.getRawAxis(1); //TODO MAP THESE BUTTONS
        double armMovement = operateJoystick.getRawAxis(5);
        double operatePOV = operateJoystick.getPOV();
        double intakeIteration = 0;
        double armIteration = 0;
        if (operatePOV == 0) {
            intakeIteration = 1;
        } else if (operatePOV == 180) {
            intakeIteration = -1;
        } else if (operatePOV == 90) {
            armIteration = 1;
        } else if (operatePOV == 270) {
            armIteration = -1;
        }
        if (operateJoystick.getRawButton(5)) {
            m_grabberSubsystem.positionGrabber(22);
        }
        if (operateJoystick.getRawButton(6)) {
            m_grabberSubsystem.positionGrabber(0);
        }
        m_armSubsystem.iterateHome(armIteration);
        m_intakeSubSystem.iterateHome(intakeIteration);
        m_armSubsystem.runArmMotors(armMovement/5);
        m_intakeSubSystem.runIntake(intakeMovement/5);
    }

    @Override 
    public boolean isFinished() {
        return (operateJoystick.getRawButton(7) && operateJoystick.getRawButton(8));
    }

    


}
