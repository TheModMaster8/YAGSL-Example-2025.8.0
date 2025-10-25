package frc.robot.subsystems.arm;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import javax.swing.plaf.metal.MetalTheme;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmSubsystem extends SubsystemBase 
{
    private final SparkMax m_masterMotor;
    private final SparkMax m_slaveMotor;
    private final AbsoluteEncoder m_encoder;
    private final SparkClosedLoopController m_pidController;

    // PID constants
    private static final double kP = 10; // 0.0001; // Adjusted for testing
    private static final double kI = 0.0001;
    private static final double kD = 0.4;
    private static final double kFF = 0.0;

    Double desiredAngle;
    Double mappedTriggerValue;
    //Boolean triggerPressed;

    public ArmSubsystem() 
    {
        // Initialize motors
        m_masterMotor = new SparkMax(7, MotorType.kBrushless);
        m_slaveMotor = new SparkMax(12, MotorType.kBrushless);

        // Create configuration objects
        SparkMaxConfig masterConfig = new SparkMaxConfig();
        SparkMaxConfig slaveConfig = new SparkMaxConfig();

        // Configure the absolute encoder (Through Bore Encoder)
        masterConfig.absoluteEncoder
            .positionConversionFactor(1.0)  // 1 rotation = 1.0 (adjust as needed)
            .velocityConversionFactor(1.0)  // Adjust as needed for your units
            .inverted(false)              // Set true if encoder reads backwards
            .zeroOffset(0.0);               // Set offset if you want a specific zero position
            
        // Configure master motor with closed-loop control
        masterConfig
            .inverted(true)                               // Makes the motor's spin direction reversed by default (invert does NOT WORK on follower/slave motors).
            .idleMode(SparkMaxConfig.IdleMode.kCoast)              // Set to coast mode for safty, if limb was crushed and E-stop pressed, robot would need to be powered off before motor's would allow rotation.
            .smartCurrentLimit(10)                      // Set current limit to 40A, setting to 5 for testing.
            .closedLoop                                            // Configure closed-loop control, this means that it will use the PID controller to reach setpoints by using the feedback sensor (ABS encoder).
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)       // Tells the motor controller to use the absolute encoder as the feedback device for closed-loop control.
            .pid(kP, kI, kD)
            .velocityFF(kFF)
            .outputRange(-1, 1);

        // Configure slave motor to follow master
        slaveConfig
            .follow(m_masterMotor,true)          // This makes the slave motor follow the master motor and makes the slave motor spin opposite of master.
            .idleMode(SparkMaxConfig.IdleMode.kCoast)   // Set to coast mode for safty, if limb was crushed and E-stop pressed, robot would need to be powered off before motor's would allow rotation.
            .smartCurrentLimit(10);          // Limits amps via software (brushless motor only), setting to 5 for testing should be 40 in final deploy.

        // Apply configurations
        m_masterMotor.configure(masterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_slaveMotor.configure(slaveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Get encoder and PID controller from master
        m_encoder = m_masterMotor.getAbsoluteEncoder();
        m_pidController = m_masterMotor.getClosedLoopController();

        // 782 at down position (hardstop)
        // 548 at up position (hardstop)
        // Means the encoder it rotating counter clockwise when going up
    }

    /** Set velocity setpoint in RPM using closed-loop control
     * @param velocityRPM Target velocity in RPM
     */
    public void setVelocity(double velocityRPM) {
        m_pidController.setReference(velocityRPM, SparkMax.ControlType.kVelocity);
    }

    /** Set position setpoint in rotations using closed-loop control
     * @param positionRotations Target position in rotations
     */
    public void setPosition(double positionRotations) {
        m_pidController.setReference(positionRotations, SparkMax.ControlType.kPosition);
    }

    /** Get current velocity in RPM */
    public double getVelocity() {
        return m_encoder.getVelocity();
    }

    /** Get current position in rotations */
    public double getPosition() {
        return m_encoder.getPosition();
    }

    /** Stop both motors */
    public void stop() {
        m_masterMotor.stopMotor();
    }

    
    public void drive(double triggerValue) 
    {
        System.out.println("drive: " + triggerValue);
        
        if (triggerValue > 0.08) // deadband to prevent jitter when trigger is not being pressed.
        {
            mappedTriggerValue = MathUtil.interpolate(0.782, 0.540, triggerValue); // Map trigger value to arm's angle range, the inversion is handled in the interpolate function (higher trigger value = lower angle = arm up).
            m_pidController.setReference(mappedTriggerValue, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0);
            System.out.println("driveTo: " + mappedTriggerValue);
        }
        /**else if (m_encoder.getPosition() > 0.8 && triggerValue <= 0.8 ) // 0.8 for some buffer, 
        {
            mappedTriggerValue = MathUtil.interpolate(0.782, 0.540, triggerValue); // Map trigger value to arm's angle range, the inversion is handled in the interpolate function (higher trigger value = lower angle = arm up).
            m_pidController.setReference(mappedTriggerValue, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0);
            System.out.println("driveTo: " + mappedTriggerValue);
        }
        else // need to have the motor turn off, as it is all the way down and no holding power needs to be applied, this would cause the motors to be constantly consuming power when it is not required or wanted.
       */
        else
        {
            stop();
        }

        System.out.println("master amp: " +  m_masterMotor.getOutputCurrent());
        System.out.println("slave amp: " +  m_slaveMotor.getOutputCurrent());
    }

    @Override
    public void periodic()
    {      
        // Make Sensor readings available on Shuffleboard
        SmartDashboard.putNumber("Arm ABS Encoder Value", m_encoder.getPosition());
        SmartDashboard.putNumber("Master Motor Amperage", m_masterMotor.getOutputCurrent());
        SmartDashboard.putNumber("Slave Motor Amperage", m_slaveMotor.getOutputCurrent());
        SmartDashboard.putNumber("Master Motor Bus Voltage", m_masterMotor.getBusVoltage());
        SmartDashboard.putNumber("Slave Motor Bus Voltage", m_slaveMotor.getBusVoltage());
    }
}