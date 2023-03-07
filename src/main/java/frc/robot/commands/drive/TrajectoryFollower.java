package frc.robot.commands.drive;

import edu.wpi.first.math.controller.HolonomicDriveController; 
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import com.pathplanner.lib.PathPlannerTrajectory;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.PathPlannerTrajectory.StopEvent;
import com.pathplanner.lib.PathPlannerTrajectory.StopEvent.ExecutionBehavior;
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

public class TrajectoryFollower extends CommandBase {
    private Drivetrain m_drivetrain; //fancy defining stuff
    PathPlannerTrajectory trajectory; //define these first up here
    HolonomicDriveController controller;
    Timer timer = new Timer();
    double startTime;
    ChassisSpeeds driveSpeeds;
    //private Drivetrain m_drivetrain;
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("New Path", new PathConstraints(4, 3));

    
    private ProfiledPIDController m_anglePID;
    public SwerveModulePosition positionFrontLeft = new SwerveModulePosition(0.0,new Rotation2d(0.0));
    public SwerveModulePosition positionFrontRight = new SwerveModulePosition(0.0,new Rotation2d(0.0));
    public SwerveModulePosition positionBackLeft = new SwerveModulePosition(0.0,new Rotation2d(0.0));
    public SwerveModulePosition positionBackRight = new SwerveModulePosition(0.0,new Rotation2d(0.0));
    public SwerveModulePosition[] initialPositions = {positionFrontLeft, positionFrontRight, positionBackLeft, positionBackRight};

    public TrajectoryFollower (Drivetrain drivetrain) { //so we can drive
        m_drivetrain = drivetrain;
        //m_drivetrain = drivetrain;
        //m_anglePID = new ProfiledPIDController(2.8, 0, 0, new TrapezoidProfile.Constraints(6.28, 3.14));
        //m_anglePID.enableContinuousInput(-180, 180);
        //PID WAS 1.6, 1.6, 2.8
        this.controller = new HolonomicDriveController(new PIDController(.8, 0, .5), new PIDController(.8, 0, .5), new ProfiledPIDController(2, 0, 0, new TrapezoidProfile.Constraints(6.28, 3.14))); //.4 is good, .85 was last P
        this.trajectory = PathPlanner.loadPath("New Path", new PathConstraints(4, 2)); //I think 4 and 4 are max velocity and acceleration //aparently we need to use "this"
       
        HashMap<String, Command> eventMap = new HashMap<>();
        //eventMap.put("BalanceCommand", new BalanceCommand(Objects.driveSubsystem));
        //m_drivetrain.m_odometry.resetPosition(trajectory.getInitialPose().getRotation(), initialPositions, trajectory.getInitialPose());
        //Objects.navx.setAngleAdjustment(trajectory.getInitialPose().getRotation().getDegrees());
        addRequirements(m_drivetrain);
    }
    
    @Override
    public void initialize () {
        // timer.reset();
        timer.start();
        System.out.println("Im initialized");
    }

    @Override
    public void execute() { //idk if this will even work
        double time = timer.get();
        PathPlannerState testState = (PathPlannerState) trajectory.sample(time);
        ChassisSpeeds driveSpeeds = controller.calculate(m_drivetrain.getCurrentPose2d(), testState, new Rotation2d(testState.holonomicRotation.getRadians())); 
        m_drivetrain.driveAutonomous(driveSpeeds);
        System.out.println("im running");
       

        //driveSpeeds.vxMetersPerSecond = -driveSpeeds.vxMetersPerSecond;
        SmartDashboard.putNumber("X Speed", driveSpeeds.vxMetersPerSecond);

//use for troubleshooting
        SmartDashboard.putNumber("time", time);
        // System.out.println(time);
        // SmartDashboard.putNumber("PosError", controller.getPosError());
        SmartDashboard.putNumber("DesiredStateX", testState.poseMeters.getX());
        SmartDashboard.putNumber("DesiredStateY", testState.poseMeters.getY());
        SmartDashboard.putNumber("DesiredRotation", testState.holonomicRotation.getDegrees());

        SmartDashboard.putNumber("XError", testState.poseMeters.getX() - m_drivetrain.getCurrentPose2d().getX());
        SmartDashboard.putNumber("YError", testState.poseMeters.getY() - m_drivetrain.getCurrentPose2d().getY());
        SmartDashboard.putNumber("RotationError", testState.holonomicRotation.getDegrees() - m_drivetrain.getCurrentPose2d().getRotation().getDegrees());
    }
    @Override
    public boolean isFinished() {
       return timer.hasElapsed(trajectory.getTotalTimeSeconds()); 

    }
       
    @Override
    public void end(boolean interrupted) {
        m_drivetrain.drive(0,0,0,true, false);
    }


}
