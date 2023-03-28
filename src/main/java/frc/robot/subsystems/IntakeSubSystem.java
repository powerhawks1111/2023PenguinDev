package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubSystem extends SubsystemBase {
    // Declaration of variables
    private int pickupPort = 12; //should be 12
    private int rotatePort = 13;
    private CANSparkMax pickupMotor = new CANSparkMax(pickupPort, MotorType.kBrushless);
    
    private CANSparkMax rotateMotor = new CANSparkMax(rotatePort, MotorType.kBrushless);
    private SparkMaxPIDController m_pidController = rotateMotor.getPIDController();
    private boolean hasCone = false;
    private double limit = 25; // Amps
    private DigitalInput m_TurnEncoderInput;
    private int rotatePWMPort = 18;
    private DutyCycle m_TurnPWMEncoder = new DutyCycle( new DigitalInput(rotatePWMPort));
    private AHRS navx = new AHRS();
    private double home = 0.356;
    private double kP = .00023;
    private double kI = 0;
    private double kD = 0;
    private double kFF = 0;
    private double kMaxOutput = 1;
    private double kMinOutput = -1;
    private double maxVel = 7000;
    private double maxAcc = 7000;
    private boolean isCalibrated = false; 
    private double calculatedMotorHome = 0; 
    private double gearRatio = 40; // was 35
    private int smartMotionSlot = 0;
    private double transformedPosition;
    private double usualHome = -52;
    private DigitalInput coneDetector;
    //private ProfiledPIDController rotatePIDController = new ProfiledPIDController(1.25, 0, 0, new Constraints(3, 1.5));
    
    


    // gear ratio is 25:1
    public IntakeSubSystem(DigitalInput input) {
        coneDetector = input;
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
      // This method will be called once per scheduler run
      //intakeMo tor.set(.25); // .55 is good
      if (m_TurnPWMEncoder.getOutput() == 0) {
        DriverStation.reportError("Make sure that the intake is connected to the Navx [Port 4]", false);
        // throw new NullPointerException(
        //     "Make sure that the intake is connected to the Navx [Port 4]"
        // );
      }
      //SmartDashboard.putNumber("Motor Amps", pickupMotor.getOutputCurrent());
      //SmartDashboard.putNumber("intake position", getIntakePosition());
      SmartDashboard.putNumber("Intake Motor Position", rotateMotor.getEncoder().getPosition());
      SmartDashboard.putNumber("Intake Raw Position", m_TurnPWMEncoder.getOutput());

    }  
    public void iterateHome(double iteration) {
        calculatedMotorHome = calculatedMotorHome + iteration;
    }

    public void brakeMode () {
        rotateMotor.setIdleMode(IdleMode.kBrake);
       
    }

    public void coastMode() {
        rotateMotor.setIdleMode(IdleMode.kCoast);
       
    }


    public double getIntakePosition() {
        return 2*Math.PI*((m_TurnPWMEncoder.getOutput() - home) % 1);
    }

    /**
     * Runs the pickup motor at a set speed
     * @param speed -1 to +1
     */
    public void runIntake (double speed) {
        pickupMotor.set(speed);
    }

    /**
     * Runs the rotate motor at a set speed
     * @param speed -1 to +1, positive is intake Down
     */
    public void rotateIntake(double speed) { //th
        rotateMotor.set(speed);
    }

    public void calculateIntakeHome () {
        // if (!(m_TurnPWMEncoder.getOutput() == 0)) {
            calculatedMotorHome = rotateMotor.getEncoder().getPosition() - (-(m_TurnPWMEncoder.getOutput() - home) * gearRatio);
            isCalibrated = true;
            SmartDashboard.putNumber("Intake Home Position", calculatedMotorHome);
        // } else {
        //     rotateMotor.getEncoder().setPosition(usualHome);
        // }
                
    }

    public boolean isCalibrated() {
        return isCalibrated;
    }

    public void setCalibrated( boolean state) {
        isCalibrated = state;
    }
    /**
     * 
     * Positions intake in degrees. 0 degrees is level
     * @param position in radians
     */
    public void positionIntake(double position) {
        position = ((- position) * (1/(2*Math.PI)) * gearRatio);

        transformedPosition = position + calculatedMotorHome;
        m_pidController.setReference(transformedPosition, CANSparkMax.ControlType.kSmartMotion, smartMotionSlot);
    }

    public void moveUp () {
        transformedPosition = transformedPosition + .1;
        m_pidController.setReference(transformedPosition, CANSparkMax.ControlType.kSmartMotion, smartMotionSlot);

    }

    public void moveDown() {
        transformedPosition = transformedPosition -.1;
        m_pidController.setReference(transformedPosition , CANSparkMax.ControlType.kSmartMotion, smartMotionSlot);
        
    }

    /**
     * Check if the pickup motor's amperage passed the set limit
     * @return boolean if the limit spiked
     */
    public boolean checkAmps () {
        return pickupMotor.getOutputCurrent() >= limit;
    }

    public boolean closeToPosition () {
        return (Math.abs(rotateMotor.getEncoder().getPosition() - transformedPosition) <= 1.6);
    
    }

    public void positionConditionally () {
        if (true) {
            positionIntake(3); //3.061
        } else {
            positionIntake(3.35);
            }
        }

    /**
     * Returns if the grabber has a cone
     * @return boolean if we have the cone
     */
    public boolean hasCone () {
        return hasCone;
    }

    /**
     * Allows other commands/subsystems to decide if we have a cone
     * @param state boolean for setting if we have the cone
     */
    public void setCone(boolean state) {
        hasCone = state;
    }
    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }


}
