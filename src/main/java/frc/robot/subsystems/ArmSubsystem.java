package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.arm.PistonArms;

public class ArmSubsystem extends SubsystemBase {
    private int rightMotor = 10; //actually 10
    private int leftMotor = 9; //acutally 9
    private int axlePWM = 19;
    private double armHomePosition = 0.441; //1907 // .441
    private CANSparkMax m_rightMotor = new CANSparkMax(rightMotor, MotorType.kBrushless); //gear ratio of motor is 155:1
    private CANSparkMax m_leftMotor = new CANSparkMax(leftMotor, MotorType.kBrushless);
    private DutyCycle encoderPWM;
    private SparkMaxPIDController m_pidController = m_rightMotor.getPIDController();
    private double kP = .000475;//.0006 works
    private double kI = 0;
    private double kD = 0;
    private double kFF = 0;
    private double kMaxOutput = 1;
    private double kMinOutput = -1;
    private double maxVel = 9000; //9000
    private double maxAcc = 9500; //9500
    private boolean isCalibrated = false; 
    private double calculatedMotorHome = 0; 
    private double gearRatio = 108; //I think 195
    private int smartMotionSlot = 0;
    private double transformedPosition;
    private boolean isScoringButDangerous = false;
    private double usualHome = 0;
    private DigitalInput coneDetector; //= new DigitalInput(9);
    private boolean m_status;


    private static DoubleSolenoid armPistons = new DoubleSolenoid(PneumaticsModuleType.REVPH, 1, 0);
    private static Compressor compressor = new Compressor(1, PneumaticsModuleType.REVPH);

    public ArmSubsystem(DigitalInput input) {
        coneDetector = input;
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
            DriverStation.reportError("Make sure that the arm is connected to the Navx [Port 5]", false);
        }
        SmartDashboard.putBoolean("ConePosition", coneDetector.get());
        isScoringButDangerous = getArmPosition() > Math.PI/2.2;
        SmartDashboard.putNumber("Axle rotation", getArmPosition());
        //SmartDashboard.putNumber("Axle rotation", encoderPWM.getOutput());

        SmartDashboard.putNumber("Motor Position", m_rightMotor.getEncoder().getPosition());
        // SmartDashboard.putNumber("Left Amps", m_leftMotor.getOutputCurrent());
        // SmartDashboard.putNumber("Right Amps", m_rightMotor.getOutputCurrent());
//        armPistons.set(DoubleSolenoid.Value.kReverse);

         //ugh right motor doesn't have an encoder
    }


    public boolean returnScoringDangerous() {
        return isScoringButDangerous;
    }
    public void setScoringButDangerous(boolean state) {
        isScoringButDangerous = state;
    }

    /**
     * returns the arm position
     * @return double radians
     */
    public double getArmPosition() { //TODO NEGATIVE CAUSE FABS BAD
        return -((2*Math.PI)*((encoderPWM.getOutput()-armHomePosition)%1));
    }
    /**
     * Run the left and right motors to move the arm
     * @param speed, positive brings the arm up and vice versa
     */

    public void runArmMotors(double speed) { 
        m_rightMotor.set(speed);
        m_leftMotor.follow(m_rightMotor, true);
    }

    /**
     * Sets the arm motors in brake mode
     */
    public void brakeMode () {
        m_leftMotor.setIdleMode(IdleMode.kBrake);
        m_rightMotor.setIdleMode(IdleMode.kBrake);
    }

    /**
     * Sets the arm motors in coast mode
     */
    public void coastMode() {
        m_leftMotor.setIdleMode(IdleMode.kCoast);
        m_rightMotor.setIdleMode(IdleMode.kCoast);
    }

    /*
    public void iterateHome(double iteration) {
        calculatedMotorHome = calculatedMotorHome + iteration;
    }

    */
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
     * Deploy the arm pistons
     * @param direction true is deploy, false is retract 
     */
    public void deployPistons(boolean direction) {
        if (direction) {
            
                armPistons.set(DoubleSolenoid.Value.kForward);
            
        } else {
           
                armPistons.set(DoubleSolenoid.Value.kReverse);
            
        }
    }

    /**
     * Set the status of whether the robot is calibrated or not
     * @param state boolean calibrated or not calibrated
     */
    public void setCalibrated( boolean state) {
        isCalibrated = state;
    } 

    /**
     * Calculates the home position of the arm position using position relative from 0 radians (resting)
     * Value is in motor rotations
     */
    public void calculateHome () {
        // if (!(encoderPWM.getOutput() == 0)) {
        calculatedMotorHome = m_rightMotor.getEncoder().getPosition() - (getArmPosition()/(2*Math.PI) * gearRatio);
        isCalibrated = true;
        SmartDashboard.putNumber("Home Motor Position", calculatedMotorHome);
        // } else {
           // m_leftMotor.getEncoder().setPosition(usualHome);

        // }
    }

    /**
     * Move the arm to its home position
     */
    public void positionConditionally () {
        positionArm(-.05);
    }

    /**
     * Return the calibrated status of the arm
     * @return calibration status
     */
    public boolean isCalibrated() {
        return isCalibrated;
    }

    /**
     * Return the status of whether the arm is close to its intended position
     */
    public boolean closeToPosition () {
        return (Math.abs(m_rightMotor.getEncoder().getPosition() - transformedPosition) <= 2.5);
    }

    public void moveUp(double change) {
        // positionArm(getArmPosition() + .1);
        transformedPosition += change;
        m_pidController.setReference(transformedPosition , CANSparkMax.ControlType.kSmartMotion, smartMotionSlot);
    }

    public void moveDown(double change) {
        // positionArm(getArmPosition() - .1);
        transformedPosition -= change;
        m_pidController.setReference(transformedPosition , CANSparkMax.ControlType.kSmartMotion, smartMotionSlot);
    }

    public void setStatus(boolean status) {
        m_status = status;
    }

    public boolean getStatus() {
        return m_status;
    }
    

    /* Deprecated (not using the beam breaker anymore)
    public boolean coneDetector() {
        return coneDetector.get();
    }
    */
}
