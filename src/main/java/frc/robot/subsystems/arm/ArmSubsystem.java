package frc.robot.subsystems.arm;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
            .inverted(false)
            .idleMode(SparkMaxConfig.IdleMode.kBrake)
            .smartCurrentLimit(5)  // Set current limit to 40A, setting to 5 for testing
            .closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pid(kP, kI, kD)
                .velocityFF(kFF)
                .outputRange(-1, 1);

        // Configure slave motor to follow master
        slaveConfig
            .follow(m_masterMotor)
            .inverted(true)  // Set to true if slave should spin opposite direction
            .idleMode(SparkMaxConfig.IdleMode.kBrake)
            .smartCurrentLimit(5);     // Set current limit to 40A, setting to 5 for testing

        // Apply configurations
        m_masterMotor.configure(masterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_slaveMotor.configure(slaveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Get encoder and PID controller from master
        m_encoder = m_masterMotor.getAbsoluteEncoder();
        m_pidController = m_masterMotor.getClosedLoopController();
    }

    /**
     * Set velocity setpoint in RPM using closed-loop control
     * @param velocityRPM Target velocity in RPM
     */
    public void setVelocity(double velocityRPM) {
        m_pidController.setReference(velocityRPM, SparkMax.ControlType.kVelocity);
    }

    /**
     * Set position setpoint in rotations using closed-loop control
     * @param positionRotations Target position in rotations
     */
    public void setPosition(double positionRotations) {
        m_pidController.setReference(positionRotations, SparkMax.ControlType.kPosition);
    }

    /**
     * Get current velocity in RPM
     */
    public double getVelocity() {
        return m_encoder.getVelocity();
    }

    /**
     * Get current position in rotations
     */
    public double getPosition() {
        return m_encoder.getPosition();
    }

    /**
     * Stop both motors
     */
    public void stop() {
        m_masterMotor.stopMotor();
    }

    public void setAngle(double desiredAngle)
    {
        desiredAngle = Math.max(Math.min(0.789, desiredAngle), 0.50);
        m_pidController.setReference(desiredAngle, SparkMax.ControlType.kPosition);
        this.desiredAngle = desiredAngle;
    }

    public void drive(double notdrive) 
    {
      setAngle(m_encoder.getPosition() + notdrive * 0.05);
    }


    @Override
    public void periodic() {
        // Add telemetry or other periodic updates here
    }
}