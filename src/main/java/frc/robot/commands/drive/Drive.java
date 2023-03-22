package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Drivetrain;

public class Drive extends CommandBase {
    public final Drivetrain m_drivetrain;
    public final Joystick m_driverJoystick;

    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(62.5);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(62.5); // was 25
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(50);
    private double driverXStick = 0;
    private double driverYStick = 0;
    private double driverRotateStick = 0;
    private boolean morePrecise = false;

    public Drive (Drivetrain drive, Joystick driverJoystick) {
        m_drivetrain = drive;
        m_driverJoystick = driverJoystick;
        addRequirements(m_drivetrain);
        
    }

    @Override
    public void execute () {
        driverXStick = m_driverJoystick.getRawAxis(1);
        driverYStick = m_driverJoystick.getRawAxis(0); //these are switched because of the goofy coordinate syste,. the x direction is actually away from driver station
        driverRotateStick = m_driverJoystick.getRawAxis(4);
        morePrecise = m_driverJoystick.getRawAxis(2) >= .2;
        double xSpeed = m_xspeedLimiter.calculate(MathUtil.applyDeadband(-driverXStick, 0.065)) * Drivetrain.kMaxSpeed; //these are all negative to correct to the feild CD system
        double ySpeed = m_yspeedLimiter.calculate(MathUtil.applyDeadband(-driverYStick, 0.065)) * Drivetrain.kMaxSpeed;
        double rot = m_rotLimiter.calculate(MathUtil.applyDeadband(-driverRotateStick, 0.065)) * Drivetrain.kMaxAngularSpeed;
        xSpeed = .25 * Math.pow(xSpeed, 3);
        ySpeed = .25 * Math.pow(ySpeed, 3);
        rot = Math.pow(rot, 3);


        // if (Math.abs(xSpeed)<.005) {
        //     xSpeed = 0;
        // }
        // if (Math.abs(ySpeed)<.005) {
        //     ySpeed = 0;
        // }      
        // if (Math.abs(rot)<.005) {
        //     rot = 0;
        // }

        //TODO MAP THIS BUTTON maybe done
        if (!morePrecise) {
            m_drivetrain.drive(xSpeed*.9, ySpeed*.9, (rot +.0001 )*1.33, true, false); //final movement; sends drive values to swerve
        } else {
            m_drivetrain.drive(xSpeed*.17, ySpeed*.17, (rot +.0001 )/3, true, false); //final movement; sends drive values to swerve

        }
    }
}
