package frc.robot.commands.auto;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.CombinedCommands.CalibrateStuff;
import frc.robot.commands.CombinedCommands.ConditionalIntake;
import frc.robot.commands.CombinedCommands.HoldTight;
import frc.robot.commands.CombinedCommands.TeleopCommands.CloseGrabber;
import frc.robot.commands.CombinedCommands.TeleopCommands.OpenGrabber;
import frc.robot.commands.CombinedCommands.TeleopCommands.ScoreCube;
import frc.robot.commands.CombinedCommands.TeleopCommands.ScoreHigh;
import frc.robot.commands.CombinedCommands.TeleopCommands.ScoreLow;
import frc.robot.commands.CombinedCommands.TeleopCommands.ScoreMid;
import frc.robot.commands.arm.ReturnHome;
import frc.robot.commands.arm.SpinGrabber;
import frc.robot.commands.arm.SpinGrabberTimed;
import frc.robot.commands.auto.AutoSpecificCommands.AutoConditional;
import frc.robot.commands.auto.AutoSpecificCommands.AutoCubeHigh;
import frc.robot.commands.auto.AutoSpecificCommands.AutoPickUpAndIntake;
import frc.robot.commands.auto.AutoSpecificCommands.AutoScoreHigh;
import frc.robot.commands.auto.AutoSpecificCommands.AutoScoreLow;
import frc.robot.commands.auto.AutoSpecificCommands.ComplicatedReturnHome;
import frc.robot.commands.auto.AutoSpecificCommands.ReverseReturnHome;
import frc.robot.commands.auto.AutoSpecificCommands.ReverseScoreHigh;
import frc.robot.commands.drive.BalanceCommand;
import frc.robot.commands.drive.StayStill;
import frc.robot.commands.intake.DefaultPositionIntake;
import frc.robot.commands.intake.IntakeCone;
import frc.robot.commands.intake.OuttakeCone;
import frc.robot.commands.intake.PositionIntake;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;

import edu.wpi.first.math.controller.HolonomicDriveController; 
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import com.pathplanner.lib.PathPlannerTrajectory;

import java.text.FieldPosition;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.PathPlannerTrajectory.StopEvent;
import com.pathplanner.lib.PathPlannerTrajectory.StopEvent.ExecutionBehavior;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.variables.Objects;
import edu.wpi.first.math.kinematics.SwerveModulePosition;


public class AutoBuilder extends CommandBase {
    List<PathPlannerTrajectory> pathGroup;
    private Drivetrain m_drivetrain;
    public IntakeSubsystem m_intakeSubSystem;
    public ArmSubsystem m_armSubsystem;
    public GrabberSubsystem m_grabberSubsystem;
    private SwerveAutoBuilder autoBuilder;
    
    Command fullAuto;
    HashMap<String, Command> eventMap = new HashMap<>();

    /**
     * Setup PathPlanner and configure all of the commands for use in PathPlanner
     * @param drivetrain
     * @param armSubsystem
     * @param intakeSubsystem
     * @param grabberSubsystem
     * @param autoPath specified auto path
     * @param maxVelocity max velocity for the specified auto path
     * @param maxAcceleration max acceleration for the specified auto
     */
    public AutoBuilder (Drivetrain drivetrain, ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, GrabberSubsystem grabberSubsystem, String autoPath, double maxVelocity, double maxAcceleration) { //TODO GET CONSTANTS RIGHT
        pathGroup = PathPlanner.loadPathGroup(autoPath, new PathConstraints(maxVelocity, maxAcceleration));
        System.out.println(autoPath);
        m_drivetrain = drivetrain;
        m_intakeSubSystem = intakeSubsystem;
        m_armSubsystem = armSubsystem;
        m_grabberSubsystem = grabberSubsystem;
        eventMap.put("turnCommand", new turnCommand(m_drivetrain));
        eventMap.put("BalanceCommand", new BalanceCommand(m_drivetrain));
        eventMap.put("hold", new HoldCommand(m_drivetrain));
        eventMap.put("ScoreHigh", new AutoScoreHigh(m_armSubsystem, m_intakeSubSystem, m_grabberSubsystem));
        eventMap.put("ScoreLow", new AutoScoreLow(m_armSubsystem, m_grabberSubsystem, m_intakeSubSystem));
        eventMap.put("ScoreMid", new ScoreMid(m_intakeSubSystem, m_armSubsystem, m_grabberSubsystem));
        eventMap.put("ReturnHome", new ReverseReturnHome(m_armSubsystem, m_intakeSubSystem, m_grabberSubsystem));
        eventMap.put("DeployIntake", new AutoConditional(m_intakeSubSystem, m_armSubsystem, m_grabberSubsystem));
        eventMap.put("StayStill", new StayStill(m_drivetrain));
        eventMap.put("HoldTight", new HoldTight(m_intakeSubSystem, m_armSubsystem, m_grabberSubsystem));
        //eventMap.put("CalibrateStuff", new CalibrateStuff(m_intakeSubSystem, m_armSubsystem));
        // eventMap.put("RunIntake", new IntakeCone(m_intakeSubSystem, m_armSubsystem, m_grabberSubsystem, new HoldTight(m_intakeSubSystem, m_armSubsystem, m_grabberSubsystem))); 
        eventMap.put("PositionIntake", new PositionIntake(m_intakeSubSystem, (Math.PI * .6666)));
        eventMap.put("SpinGrabberTimed", new SpinGrabberTimed(grabberSubsystem, .2));
        eventMap.put("SpinGrabber", new SpinGrabberTimed(grabberSubsystem, .6));
        eventMap.put("ScoreCube", new AutoCubeHigh(grabberSubsystem, intakeSubsystem, armSubsystem));
        eventMap.put("OuttakeCone", new OuttakeCone(intakeSubsystem));
        eventMap.put("PickUp", new AutoPickUpAndIntake(intakeSubsystem, armSubsystem, grabberSubsystem));
        // THESE PID TERMS WORK: Translate: P:2.75, D:.65, TURN P:1.1, D:.1
        m_drivetrain.resetNavxMark(pathGroup.get(0).getInitialHolonomicPose().getRotation().getDegrees()); //this resets our degrees to feild
        System.out.println(pathGroup.get(0).getInitialHolonomicPose().getRotation().getDegrees()); //3.25, 0, .15            was 8
        autoBuilder = new SwerveAutoBuilder(m_drivetrain::getCurrentPose2d, m_drivetrain::setPose, new PIDConstants(8, 0, 0), new PIDConstants(1.15, 0, .175), m_drivetrain::driveAutonomous, eventMap, true, m_drivetrain);
        fullAuto = autoBuilder.fullAuto(pathGroup);
        addRequirements(m_drivetrain);
        // addRequirements(m_intakeSubSystem);
        // addRequirements(m_armSubsystem);
        // addRequirements(m_grabberSubsystem);
    }
    @Override
    public void initialize () {
        SequentialCommandGroup autoSteps = new SequentialCommandGroup(new CalibrateStuff(m_intakeSubSystem, m_armSubsystem), fullAuto);
        
        autoSteps.schedule();
        //fullAuto.schedule();
        System.out.println(pathGroup.get(0).getInitialHolonomicPose().getRotation().getDegrees());
        SmartDashboard.putNumber("initRotation", pathGroup.get(0).getInitialHolonomicPose().getRotation().getDegrees());
        // Supplier<Pose2d> poseSupplier = m_driveSubsystem::getCurrentPose2d();
        // Consumer<Pose2d> poseConsumer = m_driveSubsystem::setInitialPose());
     
        //autoBuilder = new SwerveAutoBuilder(m_driveSubsystem::getCurrentPose2d, m_driveSubsystem::setInitialPose, new PIDConstants(0.8, 0, 0), new PIDConstants(2, 0, 0), m_driveSubsystem::driveAutonomous, eventMap, false, m_driveSubsystem);
    }
    @Override
    public boolean isFinished () {
        return true; 
    }
}
