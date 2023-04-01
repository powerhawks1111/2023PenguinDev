package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GrabberSubsystem extends SubsystemBase {
    private int grabMotor = 11; //should be 11
    private CANSparkMax m_grabMotor = new CANSparkMax(grabMotor, MotorType.kBrushless);
    private double limit = 8; //amps to stop the grabber calibration at, two works well
    private double openPosition = 0;
    private double range = 35;// the amount of rotations until we close the grabber
    private double cubePosition = 1;
    private double conePosition = 1;
    private double tolerance = 0.5; //the range of rotation we're happy with with the pid conroller
    private double wantedPosition = 0;
    private double transFormedPosition = 0;
    private SparkMaxPIDController m_pidController = m_grabMotor.getPIDController();
    private double kP = 1.2;
    private double kI = 0;
    private double kD = .1;
    private double kFF = 0;
    private double kMaxOutput = .5;
    private double kMinOutput = -.5;
    private boolean calibrated = true; //TODO FIX THIS

    public GrabberSubsystem() {
        m_pidController.setP(kP);
        m_pidController.setI(kI);
        m_pidController.setD(kD);
        m_pidController.setFF(kFF);
        m_pidController.setOutputRange(kMinOutput, kMaxOutput);
    }
    
    @Override
    public void periodic() {
        //SmartDashboard.putNumber("openPosition", openPosition);
        //SmartDashboard.putNumber("CurrentPosition", m_grabMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Grabber Amps", m_grabMotor.getOutputCurrent());
    }

    /**
     * Run the grabber motor at a set speed
     * Positive speed closes the grabber
     * @param speed -1 to +1
     */
    public void runGrabber(double speed) {
        m_grabMotor.set(speed);
    }

    /**
     * Checks if the grab motor's amperage passed the set limit
     * @return boolean if the limit spiked
     */
    public boolean checkGrabberAmps () {
        return m_grabMotor.getOutputCurrent() >= limit;
    }

    /**
     * Tells you if the grab motor is calibrated
     * @return boolean if the motor is calibrated
     */
    public boolean returnCalibrated() {
        return calibrated;
    }

    /**
     * Sets the grab motor's "open" encoder position
     */
    public void defineOpened() { //defines the ps
        openPosition = m_grabMotor.getEncoder().getPosition();
    }

    /**
     * Set the grab motor as calibrated or not
     * @param input boolean if the motor is calibrated
     */
    public void setCalibrated(boolean input) {
        calibrated = input;
    }

    /**
     * Spin the grab motor until it reaches its set position
     * Doesn't spin the motor if the final position is out of the range (prevents the grabber from ripping itself apart)
     * @param position set position
     */
    public void positionGrabber (double position) {
        transFormedPosition = position + openPosition; //tjis transforms everything based on the calibration. its additaion becasue adding a negative value will correct the position
        if(!(transFormedPosition < openPosition || transFormedPosition >= (openPosition + range))) { //this checks the position so we dont break the grabber
            m_pidController.setReference(transFormedPosition, CANSparkMax.ControlType.kPosition);
        }
    }   

    /**
     * Tells you if the grab motor is done moving
     * @return boolean if the grab motor can stop
     */
    public boolean closeToPosition () {
        return (Math.abs(m_grabMotor.getEncoder().getPosition() - transFormedPosition) <= tolerance);
    }
}
