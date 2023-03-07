package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.arm.PistonArms;

public class ArmSubsystem extends SubsystemBase {
    private int rightMotor = 10; //actually 10
    private int leftMotor = 9; //acutally 9
    private int axlePWM = 19;
    private double armHomePosition = 0.1907;
    private CANSparkMax m_rightMotor = new CANSparkMax(rightMotor, MotorType.kBrushless); //gear ratio of motor is 155:1
    private CANSparkMax m_leftMotor = new CANSparkMax(leftMotor, MotorType.kBrushless);
    private DutyCycle encoderPWM;
    private SparkMaxPIDController m_pidController = m_rightMotor.getPIDController();
    private double kP = .00085;
    private double kI = 0;
    private double kD = 0;
    private double kFF = 0;
    private double kMaxOutput = 1;
    private double kMinOutput = -1;
    private double maxVel = 2500;
    private double maxAcc = 1500;
    private boolean isCalibrated = false; 
    private double calculatedMotorHome = 0; 
    private double gearRatio = 155;
    private int smartMotionSlot = 0;
    private double transformedPosition;
    private boolean isScoringButDangerous = false;


    private static DoubleSolenoid armPistons = new DoubleSolenoid(PneumaticsModuleType.REVPH, 1, 0);
    private static Compressor compressor = new Compressor(1, PneumaticsModuleType.REVPH);

    public ArmSubsystem() {
        encoderPWM = new DutyCycle( new DigitalInput(axlePWM));
        m_pidController.setP(kP);
        m_pidController.setI(kI);
        m_pidController.setD(kD);
        // setIZone ?
        m_pidController.setFF(kFF);
        m_pidController.setOutputRange(kMinOutput, kMaxOutput);

        m_pidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
        m_pidController.setSmartMotionMinOutputVelocity(0, smartMotionSlot);
        m_pidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
        m_pidController.setSmartMotionAllowedClosedLoopError(.05, smartMotionSlot);// was originally uncommented
    }
    

    @Override
    public void periodic() {
        if (encoderPWM.getOutput() == 0) {
            throw new NullPointerException(
                "Make sure that the arm is connected to the Navx [Port 5]"
            );
        }
        isScoringButDangerous = getArmPosition() > Math.PI/6.2;
        SmartDashboard.putNumber("Axle rotation", getArmPosition());
        SmartDashboard.putNumber("Motor Position", m_rightMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Left Amps", m_leftMotor.getOutputCurrent());
        SmartDashboard.putNumber("Right Amps", m_rightMotor.getOutputCurrent());
//        armPistons.set(DoubleSolenoid.Value.kReverse);

         //ugh right motor doesn't have an encoder
    }

    public boolean returnScoring() {
        return isScoringButDangerous;
    }
    public void setScoringButDangerous(boolean state) {
        isScoringButDangerous = state;
    }

    /**
     * returns the arm position
     * @return double radians
     */
    public double getArmPosition() {
        return ((2*Math.PI)*(encoderPWM.getOutput()-armHomePosition));
    }
    /**
     * 
     * @param speed, positive brings the arm up
     */

    public void runArmMotors(double speed) { 
        m_rightMotor.set(speed);
        m_leftMotor.follow(m_rightMotor, true);
    }

    /**
     * 
     * @param position in radians. 0 radians is straight down
     */
    public void positionArm (double position) {
        position = position * (1/(2*Math.PI)) * gearRatio;
        transformedPosition = position + calculatedMotorHome;
        m_pidController.setReference(transformedPosition, CANSparkMax.ControlType.kSmartMotion, smartMotionSlot);
        m_leftMotor.follow(m_rightMotor, true);
    }

    /**
     * bruh
     * @param direction true is deploy arm, false is retract arm 
     */
    public void deployPistons(boolean direction) {
        
        if (direction) {
            // if (getArmPosition() > Math.PI) {
                armPistons.set(DoubleSolenoid.Value.kForward);
            // }
        } else {
            armPistons.set(DoubleSolenoid.Value.kReverse);
            
        }
    }

    public void calculateHome () {
        calculatedMotorHome = m_leftMotor.getEncoder().getPosition() - ((encoderPWM.getOutput() - armHomePosition) * gearRatio);
        isCalibrated = true;
        SmartDashboard.putNumber("Home Motor Position", calculatedMotorHome);
    }

    public boolean isCalibrated() {
        return isCalibrated;
    }
    public boolean closeToPosition () {
        return (Math.abs(m_leftMotor.getEncoder().getPosition() - transformedPosition) <= 2);
    }
}
