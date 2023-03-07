package frc.robot.commands;

import java.util.ArrayList;
//import frc.robot.commands.Command;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Scheduler {

    boolean done = false;
    boolean running = false;
    int i = 0; //index of currently running command
    ArrayList<CommandBase> commands;

    /**
     * Default constructor (commands are added on later)
     */
    public Scheduler() {}

    /**
     * Overloaded constructor that accepts an existing ArrayList of commands
     * @param coms
     * <ul><li>ArrayList of commands</li></ul>
     */
    public Scheduler(ArrayList<CommandBase> coms) {
        commands = coms;
    }

    /**
     * Runs each command in the com ArrayList
     */
    public void run() {
        CommandBase com = commands.get(i);
        if (!done) {
            com.execute();
        }
        if (!com.isFinished()) {
            running = true;
        } else {
            com.end(done);
            if (i < commands.size() - 1) {
                i++;
            } else {
                done = true;
                running = false;
            }
        }
    }

    /**
     * Accepts a list of commands
     * @param coms
     * <ul><li>ArrayList of commands</li></ul>
     */
    public void addCommands(ArrayList<CommandBase> coms) {
        commands = coms;
    }

    /**
     * Resets the scheduler to allow autonomous to run multiple times, if needed
     * <br><br>
     * Every variable is reset to it's original value
     */
    public void reset() {
        try {
            commands.clear();
            done = false;
            running = false;
            i = 0;
        } catch (Exception e) {
            return;
        }
    }
}