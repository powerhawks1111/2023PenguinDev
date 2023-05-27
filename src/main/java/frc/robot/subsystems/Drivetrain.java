// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.variables.Objects;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;




import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
/** Represents a swerve drive style drivetrain. */

public class Drivetrain extends SubsystemBase {
    
    private VisionSubsystem m_visionSubsystem;// = new VisionSubsystem();
    
    // kMaxSpeed was 2 AND kmaxangularspeed was pi/3 (before testing [district champs])
    // SOLID SPEEDS 3.25 M/S /AND PI/2.25 ROT/S
    public static final double kMaxSpeed = 3.25; // 3.68 meters per second or 12.1 ft/s (max speed of SDS Mk3 with Neo motor)
    public static final double kMaxAngularSpeed = Math.PI/2.15; // 1/2 rotation per second

    private final AHRS navx = new AHRS();
    public boolean donePlace = false;
    private PIDController xController = new PIDController(1/100, 0, 0);
    private PIDController yController = new PIDController(1/100, 0, 0);
    private double timerNow = 0;
    private double timerPrevious = 0;
    private double anglePreviousRoll = 0;
    private double anglePreviousPitch = 0;
    private double xControl = 0;
    private double yControl = 0;
    public double kPBal = .01485; //.014
    public double kDBal = 0.001;//.0019;
    //positions of each swerve unit on the robot
    // private final Translation2d m_frontLeftLocation = new Translation2d(-0.538,  0.538);
    // private final Translation2d m_frontRightLocation = new Translation2d(0.538,  0.538);
    // private final Translation2d m_backLeftLocation = new Translation2d( -0.538, -0.538);
    // private final Translation2d m_backRightLocation = new Translation2d( 0.538, -0.538);

    // //constructor for each swerve module
    // private final SwerveModule m_frontRight = new SwerveModule(21, 2, 10, 0.564);
    // private final SwerveModule m_frontLeft  = new SwerveModule(3, 4, 11, 0.563);
    // private final SwerveModule m_backLeft   = new SwerveModule(5, 6, 12, 0.875); //0.05178
    // private final SwerveModule m_backRight  = new SwerveModule(7, 8, 13, 0.921);

    // Locations pf each swerve module relative to the center of the robot
    private final Translation2d m_frontRightLocation = new Translation2d( 0.538, -0.538);
    private final Translation2d m_frontLeftLocation = new Translation2d(0.538,  0.538);
    private final Translation2d m_backLeftLocation = new Translation2d(-0.538,  0.538);
    private final Translation2d m_backRightLocation = new Translation2d( -0.538, -0.538);

    // Constructor for each swerve module
    private final SwerveModule m_frontRight  = new SwerveModule(21, 2, 10, 0.8190); //.8208
    private final SwerveModule m_frontLeft = new SwerveModule(3, 4, 11, 0.8082); //.8100
    private final SwerveModule m_backLeft  = new SwerveModule(5, 6, 12, 0.1244); //.1217
    private final SwerveModule m_backRight   = new SwerveModule(7, 8, 13, 0.1649); //.1666

    // private final SwerveModulePosition[] initialModule = new SwerveModulePosition(0, 0)
    // Swerve Drive Kinematics (note the ordering [frontRight, frontLeft, backLeft, backRight] [counterclockwise from the frontRight])
    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontRightLocation, m_frontLeftLocation, m_backLeftLocation, m_backRightLocation);


    //INITIAL POSITIONS to help define swerve drive odometry. THis was a headache
    public SwerveModulePosition positionFrontLeft = new SwerveModulePosition(0.0,new Rotation2d(0.0));
    public SwerveModulePosition positionFrontRight = new SwerveModulePosition(0.0,new Rotation2d(0.0));
    public SwerveModulePosition positionBackLeft = new SwerveModulePosition(0.0,new Rotation2d(0.0));
    public SwerveModulePosition positionBackRight = new SwerveModulePosition(0.0,new Rotation2d(0.0));
    public SwerveModulePosition[] initialPositions = {positionFrontRight, positionFrontLeft, positionBackLeft, positionBackRight};
    public SwerveDriveKinematics m_initialStates; 
    public SwerveModulePosition[] positions = new SwerveModulePosition[4];
    
    public final SwerveDriveOdometry m_odometry;
    public final SwerveDrivePoseEstimator m_visionOdometry;
    public final SwerveDrivePoseEstimator m_placeOdometry; //new SwerveDrivePoseEstimator(m_kinematics, null, null, getCurrentPose2d());
    
    
    
    // Constructor
    public Drivetrain() {
        m_visionSubsystem = new VisionSubsystem();
        m_initialStates = new SwerveDriveKinematics(m_frontRightLocation, m_frontLeftLocation, m_backLeftLocation, m_backRightLocation);

        m_placeOdometry = new SwerveDrivePoseEstimator(m_kinematics, navx.getRotation2d(), initialPositions, new Pose2d(), new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.01, 0.01, 0.01), new MatBuilder<>(Nat.N3(), Nat.N1()).fill(m_visionSubsystem.placeXStdDev, m_visionSubsystem.placeYStdDev, 0.01));

        m_visionOdometry = new SwerveDrivePoseEstimator(m_kinematics, navx.getRotation2d(), initialPositions, new Pose2d(), new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.01, 0.01, 0.01), new MatBuilder<>(Nat.N3(), Nat.N1()).fill(m_visionSubsystem.dist0XStdDev, m_visionSubsystem.dist0YStdDev, 0.01)); //These standard devs are just placeholders, we redifine in vision subsystem
        m_odometry = new SwerveDriveOdometry(
            m_kinematics, 
            navx.getRotation2d(), initialPositions
        );
        // m_visionSubsystem = new VisionSubsystem();
    }

    @Override
    public void periodic () {
        updateOdometry();
        //SmartDashboard.putNumber("xOdometry", getCurrentPose2d().getX());

    }

    /**
     * Place a cone using vision pose
     * Overrides the current odometry with the vision odometry
     */
    public void instantiatePlaceOdometry () {
        if (m_visionSubsystem.canWePlace() && m_visionSubsystem.getRelativePose()[0] != 0) {
            m_placeOdometry.resetPosition(navx.getRotation2d(), initialPositions,new Pose2d(new Translation2d(m_visionSubsystem.getRelativePose()[2], m_visionSubsystem.getVisionTags()[0]), navx.getRotation2d()));
            donePlace = true;
        } else {
            donePlace = false;
        }

    }


    public void coneAutoPlace() {
        
    }

    /**
     * Manages the positioning process for auto placing a cone
     * Uses vision odometry to control the swerve motion
     */
    public void controlAutoPlace() {
        double maxSpeed = .2;
        double yPosition = .5; //in meters
        double xPosition = .2; ///in meters
        double currentX = m_placeOdometry.getEstimatedPosition().getX();
        double currentY = m_placeOdometry.getEstimatedPosition().getY();
        double xControl = xPosition - currentX;

        double yControl;
        double rotControl = -navx.getRotation2d().getRadians()/4;

        if (Math.abs(xControl) > maxSpeed) {
            if (xControl > 0) {
                xControl = maxSpeed;
            } else {
                xControl = -maxSpeed;
            }
        }
        
        if (currentY > 0) {
            yControl = yPosition - currentY;

            if (Math.abs(yControl) > maxSpeed) {
                yControl = maxSpeed;
            }
            
            drive(xControl, -yControl/3, rotControl, true, false); // goes to either side of a node, pretty wild
        } else {
            yControl = yPosition + currentY;
            if (Math.abs(yControl) > maxSpeed) {
                yControl = -maxSpeed;
            }

            drive(xControl, -yControl, rotControl, true, false);
        }

        if (Math.abs(xControl) <= .05 && Math.abs(yControl) <= .05) {
            donePlace = false;
        }
        // drive(kMaxSpeed, kMaxAngularSpeed, anglePreviousRoll, isBalanced(), isBalanced());
    }
    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed Speed of the robot in the x direction (forward).
     * @param ySpeed Speed of the robot in the y direction (sideways).
     * @param rot Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the field.
     */
    @SuppressWarnings("ParameterName")
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean defenseHoldingMode) {
        SmartDashboard.putNumber("X Speed", xSpeed);
        SmartDashboard.putNumber("Y Speed", ySpeed);
        //double angleOffset = DriverStation.getAlliance().toString() == "Blue" ? Math.PI/2 : -Math.PI/2;
        //SmartDashboard.putString("isRed", DriverStation.getAlliance().toString());//NetworkTableInstance.getDefault().getTable("FMSInfo").getEntry("IsRedAlliance").getString("bruh"));
        // SmartDashboard.putNumber( "angleOffset", angleOffset);
        Rotation2d robotRotation = new Rotation2d(navx.getRotation2d().getRadians()); //+ angleOffset); //DriverStation.getAlliance() == Alliance.Blue ? new Rotation2d(navx.getRotation2d().getDegrees() + 180) : navx.getRotation2d();
        // SmartDashboard.putNumber ( "inputRotiation", robotRotation.getDegrees());
        var swerveModuleStates = m_kinematics.toSwerveModuleStates(fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, robotRotation): new ChassisSpeeds(xSpeed, ySpeed, rot));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
        if (!defenseHoldingMode) {
            m_frontRight.setDesiredState(swerveModuleStates[0]);
            m_frontLeft.setDesiredState(swerveModuleStates[1]);
            m_backLeft.setDesiredState(swerveModuleStates[2]);
            m_backRight.setDesiredState(swerveModuleStates[3]);
        }
        else {
            m_backLeft.setDesiredState(new SwerveModuleState(0, new Rotation2d(3*(Math.PI / 4))));
            m_frontLeft.setDesiredState(new SwerveModuleState(0, new Rotation2d( (Math.PI / 4))));
            m_backRight.setDesiredState(new SwerveModuleState(0, new Rotation2d((Math.PI / 4))));
            m_frontRight.setDesiredState(new SwerveModuleState(0, new Rotation2d(3*(Math.PI / 4))));
        }

    }

    /**
     * Balance the robot on the charging station
     * Controls the drive 
     */
    public void balanceRobot() { //Y IS NEGATIVE FOR SOME REASON
        // SmartDashboard.putNumber("Pitch", Objects.navx.getPitch());
        PIDCalculator(.02);
        drive(xControl, yControl, 0, false, false);
        
    }

    /**
     * Update the odometry and vision poses
     */
    public void setPose (Pose2d pose) {
        m_odometry.resetPosition(new Rotation2d(pose.getRotation().getRadians()), initialPositions, pose);
        m_visionOdometry.resetPosition(new Rotation2d(pose.getRotation().getRadians()), initialPositions, pose);
    }
    
    /**
     * Drives with swerve during the autonomous period
     * @param chassisSpeeds
     */
    public void driveAutonomous (ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(chassisSpeeds);
        m_frontRight.setDesiredState(moduleStates[0]);
        m_frontLeft.setDesiredState(moduleStates[1]);
        m_backLeft.setDesiredState(moduleStates[2]);
        m_backRight.setDesiredState(moduleStates[3]);
    }
    /**
     * Updates the position of the robot relative to where its starting position
     */
    public void updateOdometry() { //it may have to be in the right order
        positions[0] = new SwerveModulePosition(m_frontLeft.getDifferentState().speedMetersPerSecond, m_frontLeft.getState().angle);
        positions[1] = new SwerveModulePosition(m_backLeft.getDifferentState().speedMetersPerSecond, m_backLeft.getState().angle);
        positions[2] = new SwerveModulePosition(m_backRight.getDifferentState().speedMetersPerSecond, m_backRight.getState().angle);
        positions[3] = new SwerveModulePosition(m_frontRight.getDifferentState().speedMetersPerSecond, m_frontRight.getState().angle);
       

        Pose2d m_placeStuff =  m_placeOdometry.updateWithTime(Timer.getFPGATimestamp(), navx.getRotation2d(), positions);
        Pose2d m_distance = m_odometry.update(navx.getRotation2d(), positions);
        Pose2d m_visionDistance = m_visionOdometry.updateWithTime(Timer.getFPGATimestamp(), navx.getRotation2d(), positions);
        if (m_visionSubsystem.validVision(navx)) {
            m_visionSubsystem.visionOdometryUpdate(m_visionOdometry, navx.getRotation2d());
        }
        if (m_visionSubsystem.canWePlace()) {
            m_placeOdometry.addVisionMeasurement(new Pose2d(new Translation2d(-m_visionSubsystem.getRelativePose()[2], m_visionSubsystem.getRelativePose()[0]), navx.getRotation2d()), Timer.getFPGATimestamp());
        }

        SmartDashboard.putNumber("VisOdometry X", m_visionDistance.getX());
        SmartDashboard.putNumber("VisOdometry Y", m_visionDistance.getY());
        SmartDashboard.putNumber("VisRotation", getCurrentPose2d().getRotation().getDegrees());
        SmartDashboard.putNumber("PlaceOdometry X", m_placeStuff.getX());
        SmartDashboard.putNumber("PlaceOdometry Y", m_placeStuff.getY());
        
        SmartDashboard.putNumber("Odometry X", m_distance.getX());
        SmartDashboard.putNumber("Odometry Y", m_distance.getY());
        SmartDashboard.putNumber("Rotation", getCurrentPose2d().getRotation().getDegrees());
        // m_odometry.update(Objects.navx.getRotation2d(), new SwerveModulePosition[] {
            // m_frontRight.getState()
        // });
        // m_odometry.update(Objects.navx.getRotation2d(), m_frontLeft.getState(), m_frontRight.getState(), m_backLeft.getState(), m_backRight.getState());
     
    }

    /**
     * Reset the navx and set a starting angle
     * @param initialAngle starting angle
     */
    public void resetNavxMark (double initialAngle) {
        navx.reset(); //90 because of the feild orientation vs our driver fov
        navx.setAngleAdjustment(-initialAngle); //TODO negative because navx has a goofy coordinate system
    }
    /**
     * Gives the current position and rotation of the robot (meters) based on the wheel odometry from where the robot started
     * @return Pose2d of current robot position
     */
    public Pose2d getCurrentPose2d() {
        return m_odometry.getPoseMeters();
    }

    /**
     * Gives the current pose of the robot based on the limelight vision pose
     */
    public Pose2d visionGetCurrentPose2d() {
        return m_visionOdometry.getEstimatedPosition();
    }

    /**
     * Converts raw module states into chassis speeds
     * @return chassis speeds object
     */
    public ChassisSpeeds getChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(m_backLeft.getState(), m_frontLeft.getState(), m_backRight.getState(), m_frontRight.getState());
    }

    /**
     * Custom PID controller that observes the rate at which the pitch and roll are changing
     * @param dt double - change in time
     */
    public void PIDCalculator (double dt) {
        double thisPitch = -navx.getPitch();
        double thisRoll = navx.getRoll();
        double dPitch = thisPitch - anglePreviousPitch; 
        double dRoll = thisRoll - anglePreviousRoll;
    
        xControl = -thisPitch*kPBal; //+ (dPitch/dt) * kDBal; //xController.calculate(-Objects.navx.getPitch(), 0);//(-Objects.navx.getPitch()*kPBal); 
        yControl = -thisRoll*kPBal;// + (dRoll/dt) * kDBal;//yController.calculate(Objects.navx.getRoll(), 0);//(*kPBal);
        anglePreviousPitch = thisPitch;
        anglePreviousRoll = thisRoll;
        SmartDashboard.putNumber("PitchGyro", dPitch/dt);
        SmartDashboard.putNumber("Rollgyro", dRoll/dt);
        
    }

    /**
     * Returns whether the robot is balanced on the charging station
     * If the rate of charge of the gyro in the x or y direction is at least 10, the robot is assumed to be balanced
     * @return is the robot balanced
     */
    public boolean isBalanced() {
        // return /**(Math.abs(navx.getPitch())<= 5 && Math.abs(navx.getRoll())<=5 ) && */ (Math.abs(navx.getRawGyroX()) >= 10 || (Math.abs(navx.getRawGyroY()) >= 10));
        return Math.abs(navx.getRawGyroX()) >= 10 || (Math.abs(navx.getRawGyroY()) >= 10);
      }
}
