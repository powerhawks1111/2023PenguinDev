// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Button;



import com.pathplanner.lib.server.PathPlannerServer;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.net.PortForwarder;
import frc.robot.commands.CombinedCommands.CalibrateStuff;
import frc.robot.commands.auto.AutoBuilder;
import frc.robot.variables.Motors;
import frc.robot.variables.Objects;
import edu.wpi.first.wpilibj.Joystick;

public class Robot extends TimedRobot{
    /**
     * Main robot functions
     */
    private RobotContainer m_RobotContainer;
    private Command m_autonomousCommand;
    private Autonomous autonomous = new Autonomous();
    private Command m_autoBuilder;
    DriveAndOperate driveAndOperate = new DriveAndOperate();
    private double autoV = 2.5; //auto constraints
    private double autoA = 1.25;
    private boolean start = false;
    
    // GenericButton buttonA = new G
    

    

    /**
     * Robot variables
     */
    public String autoSelected;

    @Override
    public void robotInit() {
    
     
        // for (int port = 5800; port <= 5805; port++) {
        //     PortForwarder.add(port, "10.11.11.11", port);
        // }
        PortForwarder.add(5800, "limelight.local", 5800);
        PortForwarder.add(5801, "limelight.local", 5801);
        PortForwarder.add(5805, "limelight.local", 5805);
        CameraServer.startAutomaticCapture();
        //PathPlannerServer.startServer(5811);
        m_RobotContainer = new RobotContainer();
        m_RobotContainer.smartDashboardUpdater.setupSmartDashboard();
    

    }

    @Override
    public void robotPeriodic() {
        //SmartDashboard.putNumber("X Positiion Driving", RobotContainer.drivetrain.getCurrentPose2d().getX());
        CommandScheduler.getInstance().run();
        // if (!start) {
        //     m_RobotContainer.setDefaultCommands();
        //     start = true;
        // }
        //m_RobotContainer.visionSubsystem.updatePipeline();
       //RobotContainer.drivetrain.updateOdometry();
        // SmartDashboard.putNumber("frontRight", Objects.frontRightDuty.getOutput());
        // SmartDashboard.putNumber("frontLeft", Objects.frontLeftDuty.getOutput());
        // SmartDashboard.putNumber("backLeft", Objects.backLeftDuty.getOutput());
        // SmartDashboard.putNumber("backRight", Objects.backRightDuty.getOutput());
    }

    @Override
    public void autonomousInit() {
        // autoSelected = RobotContainer.smartDashboardUpdater.getAutoSelected();
    //    m_autonomousCommand = m_RobotContainer.getAutoCommand();
    //    m_autonomousCommand.schedule();
        //m_RobotContainer.setDefaultCommands();
        m_RobotContainer.setDefaultCommands();
        m_RobotContainer.getAutoCommand().schedule();
        
       
        

    }

    @Override
    public void autonomousPeriodic() {
        
        // switch (autoSelected) {
        //     case "Red Path":
        //         autonomous.redPath();
        //         break;
        //     case "Orange Path":
        //         autonomous.orangePath();
        //         break;
        //     case "Yellow Path":
        //         autonomous.yellowPath();
        //         break;
        //     case "Green Path":
        //         autonomous.greenPath();
        //         break;
        //     case "Blue Path":
        //         autonomous.bluePath();
        //         break;
        //     case "Purple Path":
        //         autonomous.purplePath();
        //         break;
        //     case "Default Path":
        //     default:
        //         autonomous.defaultPath();
        //         break;
        // }
        
    }

    @Override
    public void teleopInit() {
        // if (m_autonomousCommand != null) {
        //     m_autonomousCommand.cancel();
        // }
        

        CommandScheduler.getInstance().cancelAll();
        m_RobotContainer.setDefaultCommands();
        


        //m_RobotContainer.defaultCommand.schedule();
    }

    @Override
    public void teleopPeriodic() {
        
        // driveAndOperate.readDriverController();
        // driveAndOperate.readOperatorController();
        // // driveAndOperate.testJoystickRead();
        // driveAndOperate.driveAndOperate();
        //SmartDashboard.putNumber("xVisions", m_RobotContainer.visionSubsystem.getVisionTags(2)[1]);
       
        
    }


}
