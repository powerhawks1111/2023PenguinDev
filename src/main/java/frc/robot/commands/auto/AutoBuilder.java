package frc.robot.commands.auto;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.drive.BalanceCommand;
import frc.robot.subsystems.DriveSubsystem;

import edu.wpi.first.math.controller.HolonomicDriveController; 
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Drivetrain;
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


public class AutoBuilder extends CommandBase{

    List<PathPlannerTrajectory> pathGroup;
    private Drivetrain m_drivetrain;
    private SwerveAutoBuilder autoBuilder;
    Command fullAuto;
    HashMap<String, Command> eventMap = new HashMap<>();
    

    public AutoBuilder ( Drivetrain drivetrain, String autoPath, double maxVelocity, double maxAcceleration) { //TODO GET CONSTANTS RIGHT
        pathGroup = PathPlanner.loadPathGroup(autoPath, new PathConstraints(maxVelocity, maxAcceleration));
        m_drivetrain = drivetrain;
        eventMap.put("turnCommand", new turnCommandTest(m_drivetrain));
        eventMap.put("balance", new BalanceCommand(m_drivetrain));
        eventMap.put("hold", new HoldCommand(m_drivetrain));
        // THESE PID TERMS WORK: Translate: P:2.75, D:.65, TURN P:1.1, D:.1
        m_drivetrain.resetNavxMark(pathGroup.get(0).getInitialHolonomicPose().getRotation().getDegrees()); //this resets our degrees to feild
        System.out.println(pathGroup.get(0).getInitialHolonomicPose().getRotation().getDegrees());
        autoBuilder = new SwerveAutoBuilder(m_drivetrain::visionGetCurrentPose2d, m_drivetrain::setPose, new PIDConstants(3.25, 0, .15), new PIDConstants(1.15, 0, .175), m_drivetrain::driveAutonomous, eventMap, true, m_drivetrain);
        fullAuto = autoBuilder.fullAuto(pathGroup);
        addRequirements(m_drivetrain);
    }
    @Override
    public void initialize () {
        fullAuto.schedule();
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
