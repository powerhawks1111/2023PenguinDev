package frc.robot;


import javax.swing.text.Position;

import org.ejml.simple.SimpleMatrix;

import com.kauailabs.navx.frc.AHRS; 
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Direction;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.*;

import frc.robot.DriveAndOperate;
import frc.robot.SmartDashboardUpdater;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeSubSystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.commands.Scheduler;
import frc.robot.commands.CombinedCommands.CalibrateStuff;
// import frc.robot.commands.CombinedCommands.ComplexScore;
import frc.robot.commands.CombinedCommands.ConditionalIntake;
import frc.robot.commands.CombinedCommands.HoldTight;
import frc.robot.commands.CombinedCommands.PickupAndRotate;
import frc.robot.commands.CombinedCommands.TeleopCommands.CloseGrabber;
import frc.robot.commands.CombinedCommands.TeleopCommands.OpenGrabber;
import frc.robot.commands.CombinedCommands.TeleopCommands.ScoreHigh;
import frc.robot.commands.CombinedCommands.TeleopCommands.ScoreLow;
import frc.robot.commands.CombinedCommands.TeleopCommands.ScoreMid;
import frc.robot.commands.arm.CalibrateArm;
import frc.robot.commands.arm.ControlArm;
import frc.robot.commands.arm.PistonArms;
import frc.robot.commands.arm.PositionGrabber;
import frc.robot.commands.arm.Score;
import frc.robot.commands.auto.AutoBuilder;
import frc.robot.commands.auto.simpleTrajectory;
import frc.robot.commands.drive.BalanceCommand;
import frc.robot.commands.drive.Drive;
import frc.robot.commands.intake.CalibrateIntake;
import frc.robot.commands.intake.DefaultPositionIntake;
import frc.robot.commands.intake.IntakeCone;
import frc.robot.commands.intake.PositionIntake;
import edu.wpi.first.wpilibj.Compressor;

public class RobotContainer {
    //public final  AHRS navx = new AHRS();
    
    //public final Scheduler scheduler = new Scheduler();
    /**
     * ---------------------------------------------------------------------------------------
     * Subsystems
     * ---------------------------------------------------------------------------------------
     */
   // public final Drivetrain drivetrain = new Drivetrain();
    public final Drivetrain drivetrain = new Drivetrain();
    public final IntakeSubSystem intakeSubSystem = new IntakeSubSystem();
    public final ArmSubsystem armSubsystem = new ArmSubsystem();
    public final GrabberSubsystem grabberSubsystem = new GrabberSubsystem();
    
    public final VisionSubsystem visionSubsystem = new VisionSubsystem();
    public final static SmartDashboardUpdater smartDashboardUpdater = new SmartDashboardUpdater();

    
    public Joystick driveJoystick = new Joystick(0);
    public String autoSelected;

    public Command m_autoBuilder;
    //endocers
    //to see duty cycle stuff on smart dashboard ot align wheels
    // public static DigitalInput frontRightInput = new DigitalInput(10);
    // public static DutyCycle frontRightDuty = new DutyCycle(frontRightInput);

    // public static DigitalInput frontLeftInput = new DigitalInput(11);
    // public static DutyCycle frontLeftDuty = new DutyCycle(frontLeftInput );

    // public static DigitalInput backLeftInput = new DigitalInput(12);
    // public static DutyCycle backLeftDuty = new DutyCycle(backLeftInput);

    // public static DigitalInput backRightInput = new DigitalInput(13);
    // public static DutyCycle backRightDuty = new DutyCycle(backRightInput);


    //private Command simpleTrajectory = new simpleTrajectory(drivetrain);
    //public Command defaultCommand = new Drive(drivetrain, driveJoystick);

    public Command getAutoCommand () {
        autoSelected = RobotContainer.smartDashboardUpdater.getAutoSelected();
        if (autoSelected == "Red Path") { //we have to modify these path constraints to best fit the paths, they'll change with each path
            m_autoBuilder = new AutoBuilder(drivetrain, autoSelected, 2, 1);
            return m_autoBuilder;
        } else if (autoSelected == "Orange Path") {
            m_autoBuilder = new AutoBuilder(drivetrain, autoSelected, 2, 1);
            return m_autoBuilder;
        } else if (autoSelected == "Yellow Path") {
            m_autoBuilder = new AutoBuilder(drivetrain, autoSelected, 2, 1);
            return m_autoBuilder;
        } else if (autoSelected == "Green Path") {
            m_autoBuilder = new AutoBuilder(drivetrain, autoSelected, 2, 1);
            return m_autoBuilder;
        } else if (autoSelected == "Blue Path") {
            m_autoBuilder = new AutoBuilder(drivetrain, autoSelected, 2, 1);
            return m_autoBuilder;
        } else if (autoSelected == "Purple Path") {
            m_autoBuilder = new AutoBuilder(drivetrain, autoSelected, 2, 1);
            return m_autoBuilder;
        } else {
            m_autoBuilder = new AutoBuilder(drivetrain, "Default Command", 2, 1);
            return m_autoBuilder;
        }
    }
            
  
  
    public RobotContainer () {
        configureButtonBindings();
        setDefaultCommands ();
        //driveSubsystem.setDefaultCommand(new Drive(driveSubsystem, driveJoystick));
    }


    public void configureButtonBindings () {
        new JoystickButton(driveJoystick, 6).whileTrue(new ConditionalIntake(intakeSubSystem, armSubsystem, grabberSubsystem, new HoldTight(intakeSubSystem, armSubsystem, grabberSubsystem))); //DONT DO DEGREES
        new JoystickButton(driveJoystick, 2).whileTrue(new ScoreLow(intakeSubSystem, armSubsystem));
        new JoystickButton(driveJoystick, 1).whileTrue(new ScoreMid(intakeSubSystem, armSubsystem));
        new JoystickButton(driveJoystick, 4).whileTrue(new ScoreHigh(intakeSubSystem, armSubsystem));
        new JoystickButton(driveJoystick, 5).whileTrue(new CloseGrabber(grabberSubsystem));
        new JoystickButton(driveJoystick, 3).whileTrue(new OpenGrabber(grabberSubsystem));



        // new JoystickButton(driveJoystick, 2).whileTrue(new PositionGrabber(grabberSubsystem, 0)); //DONT DO DEGREES
        //new JoystickButton(driveJoystick, 3).whileTrue(new PistonArms(armSubsystem, true)); 
        // new JoystickButton(driveJoystick, 4).whileTrue(new ControlArm(armSubsystem, 0));
    }

    // Set default/passive commands for each subsystem
    public void setDefaultCommands () {
        Command command = new CalibrateStuff(intakeSubSystem, armSubsystem); //calibrates everything
        command.schedule();
        //drivetrain.setDefaultCommand(new Drive(drivetrain, driveJoystick));
        intakeSubSystem.setDefaultCommand(new DefaultPositionIntake(intakeSubSystem, Math.PI*(.66)));
        //armSubsystem.setDefaultCommand(new PistonArms(armSubsystem, false));
    }
    /**
     * ---------------------------------------------------------------------------------------
     * Climb
     * ---------------------q------------------------------------------------------------------
     */
 //intentionally reversed

}
