package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.TrajectoryFollower;
import frc.robot.variables.Objects;

import java.util.ArrayList;

public class Autonomous {

    private ArrayList<CommandBase> commandsList;
    boolean start = false;
    
    public Autonomous() {
        commandsList = new ArrayList<CommandBase>();
    }

    public void redPath() { //Original FOUR BALL AUTO from comp 2
        if(!start) {
            
           
            start = true;
        }
        startAutonomous();
    }

    public void orangePath() { //THREE BALL, temporarily commented out for path following testing
        if (!start) {
            //commandsList.add(new TrajectoryFollower(Objects.driveSubsystem, Objects.drivetrain));
            // commandsList.add(new ResetCommand(90));
            // commandsList.add(new HoodZeroCommand(Objects.hoodSubsystem));
            // commandsList.add(new MoveToCommand(Objects.moveToSubsystem, 40, -1, 90, 0.7, .6, 10, true)); //FIRST BALL
            // // commandsList.add(new HoodZeroCommand(Objects.hoodSubsystem));
            // // commandsList.add(new MoveToCommand(Objects.moveToSubsystem, 37, 0, 90, 0.5, 0.15));
            // //commandsList.add(new IntakeCommand(Objects.intakeAuto));
            // commandsList.add(new ReadyToShootCommand());
            // commandsList.add (new ShootCommand(Objects.shootSubsystem, Objects.hoodSubsystem, Objects.drivetrain, Objects.visionSubsystem, false , Objects.visionSubsystem.rpmFromVision(), false));
            // commandsList.add(new ReadyToShootCommand());
            // commandsList.add (new ShootCommand(Objects.shootSubsystem, Objects.hoodSubsystem, Objects.drivetrain, Objects.visionSubsystem, false, Objects.visionSubsystem.rpmFromVision(), true));
            
            // //commandsList.add (new MoveToCommand(Objects.moveToSubsystem, -10, -90, 180, .65, .95, 10, true, false)); //SECOND BALL //TODO REMOVE THIS

            // commandsList.add(new ReadyToShootCommand());
            // commandsList.add (new ShootCommand(Objects.shootSubsystem, Objects.hoodSubsystem, Objects.drivetrain, Objects.visionSubsystem, false, Objects.visionSubsystem.rpmFromVision(), false));
            
            start=true;
        }
        startAutonomous();

    }

    public void yellowPath() { //TWO BALL in four ball position
    if (!start) {
       
        start = true;
        }
            startAutonomous();
    }

    public void greenPath() { //TWO BALL with one ball denial in two ball position (BOTTOM LEFT)
        if (!start) {
            
            start = true;
        }
        startAutonomous();
    }

    public void bluePath() { //TWO BALL with TWO BALL denail
        if(!start) {
           
            start = true;
        }
        startAutonomous();
    }

    public void purplePath() { //5 ball
        if(!start) {
           
            //commandsList.add(new HoodZeroCommand(Objects.hoodSubsystem));
            // comman54dsList.add(new MoveToCommand(Objects.moveToSubsystem, -35, 3, 0, 0.4, 0.25));
            // commandsList.add(new MoveToCommand(Objects.moveToSubsystem,0 , 30, 0, 0.5, 0.25));
            

            start = true;
        }
        startAutonomous();
    }

    public void defaultPath() {

    }

    public void startAutonomous() {
        
    }
}
