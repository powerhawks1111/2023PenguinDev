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

    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(10);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(10);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(10);
    private double driverXStick = 0;
    private double driverYStick = 0;
    private double driverRotateStick = 0;

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
        double xSpeed = m_xspeedLimiter.calculate(MathUtil.applyDeadband(-driverXStick, 0.05)) * Drivetrain.kMaxSpeed; //these are all negative to correct to the feild CD system
        double ySpeed = m_yspeedLimiter.calculate(MathUtil.applyDeadband(-driverYStick, 0.05)) * Drivetrain.kMaxSpeed;
        double rot = m_rotLimiter.calculate(-MathUtil.applyDeadband(driverRotateStick, 0.05)) * Drivetrain.kMaxAngularSpeed;



        if (Math.abs(xSpeed)<.015) {
            xSpeed = 0;
        }
        if (Math.abs(ySpeed)<.015) {
            ySpeed = 0;
        }      
        if (Math.abs(rot)<.015) {
            rot = 0;
        }

        m_drivetrain.drive(xSpeed*.3, ySpeed*.3, (rot +.0001 )/2, true, false); //final movement; sends drive values to swerve
    }
}
