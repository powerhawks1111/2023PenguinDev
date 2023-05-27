package frc.robot;


import java.util.function.BooleanSupplier;

import javax.swing.text.Position;

import org.ejml.simple.SimpleMatrix;

import com.kauailabs.navx.frc.AHRS; 
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Direction;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.commands.Scheduler;
import frc.robot.commands.CombinedCommands.CalibrateStuff;
// import frc.robot.commands.CombinedCommands.ComplexScore;
import frc.robot.commands.CombinedCommands.ConditionalIntake;
import frc.robot.commands.CombinedCommands.EjectCone;
import frc.robot.commands.CombinedCommands.HoldTight;
import frc.robot.commands.CombinedCommands.HumanPlayerReturnHome;
import frc.robot.commands.CombinedCommands.HumanPlayerStation;
import frc.robot.commands.CombinedCommands.PlaceCommand;
import frc.robot.commands.CombinedCommands.TeleopCommands.CloseGrabber;
import frc.robot.commands.CombinedCommands.TeleopCommands.DoubleStation;
import frc.robot.commands.CombinedCommands.TeleopCommands.OpenGrabber;
import frc.robot.commands.CombinedCommands.TeleopCommands.OuttakeBackwards;
import frc.robot.commands.CombinedCommands.TeleopCommands.ScoreCone;
import frc.robot.commands.CombinedCommands.TeleopCommands.ScoreHigh;
import frc.robot.commands.CombinedCommands.TeleopCommands.ScoreLow;
import frc.robot.commands.CombinedCommands.TeleopCommands.ScoreMid;
import frc.robot.commands.CombinedCommands.TeleopCommands.TeleopReturnHome;
import frc.robot.commands.arm.CalibrateArm;
import frc.robot.commands.arm.ControlArm;
import frc.robot.commands.arm.MoveArm;
import frc.robot.commands.arm.MoveArmDown;
import frc.robot.commands.arm.MoveArmUp;
import frc.robot.commands.arm.PassiveCubeIntake;
import frc.robot.commands.arm.PistonArms;
import frc.robot.commands.arm.PositionGrabber;
import frc.robot.commands.arm.ReleaseConeFast;
import frc.robot.commands.arm.ReleaseConeSlow;
import frc.robot.commands.arm.ReturnHome;
import frc.robot.commands.arm.Score;
import frc.robot.commands.arm.SpinGrabber;
import frc.robot.commands.arm.SpinGrabberTimed;
import frc.robot.commands.auto.AutoBuilder;
import frc.robot.commands.auto.simpleTrajectory;
import frc.robot.commands.auto.AutoSpecificCommands.AutoScoreHigh;
import frc.robot.commands.auto.AutoSpecificCommands.ReverseReturnHome;
import frc.robot.commands.drive.AutoPlace;
import frc.robot.commands.drive.BalanceCommand;
import frc.robot.commands.drive.Drive;
import frc.robot.commands.drive.OperatorOverride;
import frc.robot.commands.intake.CalibrateIntake;
import frc.robot.commands.intake.DefaultPositionIntake;
import frc.robot.commands.intake.IntakeCone;
import frc.robot.commands.intake.OuttakeCone;
import frc.robot.commands.intake.PositionIntake;
import frc.robot.commands.intake.ResetConeStatus;
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
    public DigitalInput m_condeDetector = new DigitalInput(9);

    public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(m_condeDetector);
    public final ArmSubsystem armSubsystem = new ArmSubsystem(m_condeDetector);
    public final GrabberSubsystem grabberSubsystem = new GrabberSubsystem();
    
    public final VisionSubsystem visionSubsystem = new VisionSubsystem();
    public final SmartDashboardUpdater smartDashboardUpdater = new SmartDashboardUpdater();

    
    public Joystick driveJoystick = new Joystick(0);
    public Joystick operateJoystick = new Joystick(1);
    public String autoSelected;

    public Trigger left = new JoystickButton(operateJoystick, 7);
    public Trigger right = new JoystickButton(operateJoystick, 8);
    //public Trigger  intakeTrigger = new Trigger(new BooleanEvent(null, left));
        
    public Trigger rightTrigger = new RightTrigger();

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
        autoSelected = smartDashboardUpdater.getAutoSelected();
        if (autoSelected == "Red Path") { //we have to modify these path constraints to best fit the paths, they'll change with each path
            m_autoBuilder = new AutoBuilder(drivetrain, armSubsystem, intakeSubsystem, grabberSubsystem, autoSelected, 2.2, 1.75);
            intakeSubsystem.setCone(false); // cube auto (hehe)
            return m_autoBuilder;
        } else if (autoSelected == "Orange Path") {
            m_autoBuilder = new AutoBuilder(drivetrain, armSubsystem, intakeSubsystem, grabberSubsystem,autoSelected, 1.75, 1.25);
            intakeSubsystem.setCone(true);

            return m_autoBuilder;
        } else if (autoSelected == "Yellow Path") {
            m_autoBuilder = new AutoBuilder(drivetrain, armSubsystem, intakeSubsystem, grabberSubsystem,autoSelected, 1.5, 1);
            intakeSubsystem.setCone(true);

            return m_autoBuilder;
        } else if (autoSelected == "Green Path") {
            m_autoBuilder = new AutoBuilder(drivetrain, armSubsystem, intakeSubsystem, grabberSubsystem,autoSelected, 2.2, 1.75);
            intakeSubsystem.setCone(true);

            return m_autoBuilder;
        } else if (autoSelected == "Blue Path") {
            m_autoBuilder = new AutoBuilder(drivetrain, armSubsystem, intakeSubsystem, grabberSubsystem,autoSelected, 2.75, 1.2);
            intakeSubsystem.setCone(true);

            return m_autoBuilder;
        } else if (autoSelected == "Purple Path") {
            m_autoBuilder = new AutoBuilder(drivetrain, armSubsystem, intakeSubsystem, grabberSubsystem,autoSelected, 2.5, 1.5);
            intakeSubsystem.setCone(true);

            return m_autoBuilder;
        } else if (autoSelected == "Teal Path") {
            m_autoBuilder = new AutoBuilder(drivetrain, armSubsystem, intakeSubsystem, grabberSubsystem, autoSelected, 2.5, 1.5);
            intakeSubsystem.setCone(true);
        } else {
            m_autoBuilder = new AutoBuilder(drivetrain, armSubsystem, intakeSubsystem, grabberSubsystem, "Default Path", 2.5, 1.75); // 2.65, 1.75
            intakeSubsystem.setCone(true);
        }
        return m_autoBuilder;
    }
            
  
  
    public RobotContainer () {
        configureButtonBindings();
        //setDefaultCommands ();
        //driveSubsystem.setDefaultCommand(new Drive(driveSubsystem, driveJoystick));
    }


    public void configureButtonBindings () {
        new JoystickButton(driveJoystick, 6).whileTrue(new ConditionalIntake(intakeSubsystem, armSubsystem, grabberSubsystem)); //DONT DO DEGREES
        new JoystickButton(driveJoystick, 9).onTrue(new MoveArm(armSubsystem, 5.5));
        // new JoystickButton(driveJoystick, 9).onTrue(new MoveArmDown(armSubsystem, 5.5));
        // new JoystickButton(driveJoystick, 9).onFalse(new MoveArmUp(armSubsystem, 5.5));

        // new JoystickButton(driveJoystick, 9).onTrue(new MoveArmDown(armSubsystem, 2));
        // new JoystickButton(driveJoystick, 9).onFalse(new MoveArmUp(armSubsystem, 2));

        new JoystickButton(driveJoystick, 2).whileTrue(new BalanceCommand(drivetrain));
        new JoystickButton(driveJoystick, 5).whileTrue(new EjectCone(intakeSubsystem, armSubsystem, grabberSubsystem));
        new JoystickButton(driveJoystick, 3).whileTrue(new HumanPlayerStation(intakeSubsystem, armSubsystem, grabberSubsystem, new HoldTight(intakeSubsystem, armSubsystem, grabberSubsystem)));
        new JoystickButton(driveJoystick, 3).onFalse(new HumanPlayerReturnHome(armSubsystem, intakeSubsystem));
        new JoystickButton(driveJoystick, 7).onTrue(new ResetConeStatus(intakeSubsystem));
        new JoystickButton(driveJoystick, 8).onTrue(new PassiveCubeIntake(grabberSubsystem, intakeSubsystem, operateJoystick, driveJoystick));
        
        
        // new JoystickButton(driveJoystick, 3).onTrue(new AutoScoreHigh(armSubsystem, intakeSubSystem, grabberSubsystem));
        // new JoystickButton(driveJoystick, 4).onTrue(new ReverseReturnHome(armSubsystem, intakeSubSystem, grabberSubsystem));
 

        // NEW COMMANDS TODO CHANGE BUTTON

        
        

        //new JoystickButton(driveJoystick, 5).whileTrue(new AutoPlace(drivetrain));
        
        // // .whileTrue(new EjectCone(intakeSubSystem));
        




        
        new JoystickButton(operateJoystick, 1).onTrue(new ScoreLow(intakeSubsystem, armSubsystem, grabberSubsystem));
        new JoystickButton(operateJoystick, 3).onTrue(new ScoreMid(intakeSubsystem, armSubsystem, grabberSubsystem));
        new JoystickButton(operateJoystick, 4).onTrue(new ScoreHigh(intakeSubsystem, armSubsystem, grabberSubsystem));
        new JoystickButton(operateJoystick, 2).onTrue(new TeleopReturnHome(armSubsystem, grabberSubsystem));
        new JoystickButton(operateJoystick, 5).whileTrue(new DoubleStation(armSubsystem, grabberSubsystem));
        new JoystickButton(operateJoystick, 6).onTrue(new ReleaseConeFast(grabberSubsystem));
        new JoystickButton(operateJoystick, 10).onTrue(new HoldTight(intakeSubsystem, armSubsystem, grabberSubsystem));
        // new JoystickButton(operateJoystick, 9).whileTrue(new HumanPlayerStation(intakeSubSystem, armSubsystem, grabberSubsystem, new HoldTight(intakeSubSystem, armSubsystem, grabberSubsystem)));
        // new JoystickButton(operateJoystick, 9).onFalse(new HumanPlayerReturnHome(armSubsystem, intakeSubSystem));
        left.and(right).toggleOnTrue(new OperatorOverride(armSubsystem, intakeSubsystem, grabberSubsystem, operateJoystick));






        // new JoystickButton(operateJoystick, 1).onTrue(new SpinGrabber(grabberSubsystem, 1));
        // new JoystickButton(operateJoystick, 2).onTrue(new ReleaseCone(grabberSubsystem));
        // new JoystickButton(operateJoystick, 3).whileTrue(new ConditionalIntake(intakeSubSystem, armSubsystem, grabberSubsystem)); //DONT DO DEGREES
        // new JoystickButton(operateJoystick, 4).onTrue(new HoldTight(intakeSubSystem, armSubsystem, grabberSubsystem));
        // new JoystickButton(operateJoystick, 5).onTrue(new ScoreLow(intakeSubSystem, armSubsystem, grabberSubsystem));
        // new JoystickButton(operateJoystick, 6).onTrue(new ScoreMid(intakeSubSystem, armSubsystem, grabberSubsystem));
        // new JoystickButton(operateJoystick, 7).onTrue(new TeleopReturnHome(armSubsystem, grabberSubsystem));



        //new JoystickButton(operateJoystick, 8).onTrue(new OuttakeBackwards(intakeSubSystem));
        // rightTrigger.whileTrue(m_autoBuilder)
        //new JoystickButton(operateJoystick, 7, 8).onTrue(new HoldTight(intakeSubSystem, armSubsystem, grabberSubsystem));

        
        
        
        //new JoystickButton(driveJoystick, 6).whileTrue(new OpenGrabber(grabberSubsystem));
        

        // new JoystickButton(driveJoystick, 2).whileTrue(new PositionGrabber(grabberSubsystem, 0)); //DONT DO DEGREES
        //new JoystickButton(driveJoystick, 3).whileTrue(new PistonArms(armSubsystem, true)); 
        // new JoystickButton(driveJoystick, 4).whileTrue(new ControlArm(armSubsystem, 0));
    }

    // Set default/passive commands for each subsystem
    public void setDefaultCommands () {
        Command command = new CalibrateStuff(intakeSubsystem, armSubsystem); //calibrates everything
        command.schedule();
        drivetrain.setDefaultCommand(new Drive(drivetrain, driveJoystick));
        intakeSubsystem.setDefaultCommand(new DefaultPositionIntake(intakeSubsystem, Math.PI*(.66)));
        //armSubsystem.setDefaultCommand(new PistonArms(armSubsystem, false));
    }
  
    class RightTrigger extends Trigger {
        // @Override
        public boolean getAsBoolean() {
            return (driveJoystick.getRawAxis(3) > .2);
        }
    }

}


