package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.Drive;
import frc.robot.variables.Objects;

public class DriveSubsystem extends SubsystemBase {

    /*
     * THIS FILE IS NOT BEING USED 
     * REFER to Drivetrain.java
     */

   
    // public final Drivetrain m_drive = new Drivetrain();
    // public final AHRS m_navx = new AHRS();
    

    public DriveSubsystem () {
       
    }
    /* 
    
    public void driveSwerve(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean defenseHoldingMode) {
        m_drive.drive(xSpeed, ySpeed, rot, fieldRelative, defenseHoldingMode);
    }
    */

    @Override 
    public void periodic () {
        //updateOdometry();
        //SmartDashboard.putNumber("xOdometry", getCurrentPose2d().getX());
    }
    //TODO FIX ANGEL INIT
    /*
    public void setInitialPose (Pose2d initPose2d) {
        m_drive.setPose( initPose2d);
    }
    */


/* 

    public double[] translateToPosition(double desiredPositionX, double desiredPositionY, double speedScale) {
        double xDisplace =  m_drive.getCurrentPose2d().getX();
        double yDisplace = m_drive.getCurrentPose2d().getY();
        SmartDashboard.putNumber("Xposition", xDisplace);
        SmartDashboard.putNumber("YPosition", yDisplace);
        double xMovement = desiredPositionX - xDisplace;
        double yMovement = desiredPositionY - yDisplace;
        double[] componentSpeeds = {0,0};
        if (Math.abs(xMovement) >= 10 ) {
            if (xMovement<0) { 
            xMovement = -speedScale;
            } else {
            xMovement = speedScale;
            }
            componentSpeeds[0] = xMovement;
        } else {
            componentSpeeds[0] =  Math.pow(speedScale * xMovement/10, 3);
        }

        if (Math.abs(yMovement) >= 10 ) {
            if (yMovement<0) {
                yMovement = -speedScale;
            } else {
                yMovement = speedScale;
            }
            componentSpeeds[1] = yMovement;    
        } else {
            componentSpeeds[1] = Math.pow(speedScale * yMovement/10, 3);
        }
        return componentSpeeds;
    }
*/  /* 
    public void driveAutonomous(ChassisSpeeds speeds) {
        m_drive.driveAutonomous(speeds);
    }
    */
    
    // public Pose2d visionGetCurrentPose2d() {
    //     return m_drive.visionGetCurrentPose2d();
    // }

    // public void resetNavxMark (double initialAngle) {
    //     m_drive.resetNavxMark(initialAngle);
    // }
    /* 
    public void updateOdometry () {
        m_drive.updateOdometry();

    } 
    public Pose2d getCurrentPose2d () {
        return m_drive.getCurrentPose2d();
    }

    
    public boolean isBalanced() {
      return (Math.abs(m_navx.getPitch())<= 5 && Math.abs(m_navx.getRoll())<=5 ) && (Math.abs(m_navx.getRawGyroX()) <= 5 && (Math.abs(m_navx.getRawGyroY()) <= 5));
    }
    */
}
