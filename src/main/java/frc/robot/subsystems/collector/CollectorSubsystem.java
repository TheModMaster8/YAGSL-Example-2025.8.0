package frc.robot.subsystems.collector;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
//import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class CollectorSubsystem extends SubsystemBase
{
    // LedManagerSubsystem ledManager;
    DigitalInput pieceSensor;
    // DutyCycleEncoder pieceSensor;
    private final SparkMax m_collectorMotor;

    public CollectorSubsystem() 
    {
        m_collectorMotor = new SparkMax(9, MotorType.kBrushed);
        
        // Create configuration objects
        SparkMaxConfig collectorConfig = new SparkMaxConfig();
        
        collectorConfig
            .inverted(true)                               // Makes the motor's spin direction reversed by default (invert does NOT WORK on follower/slave motors).
            .idleMode(SparkMaxConfig.IdleMode.kCoast)              // Set to coast mode for safty, if something gets caught in the rollers like a necklace, clothes, cord, and E-stop is pressed, robot would need to be powered off before motor's would allow rotation, "should" prevent strangulation. .
            .smartCurrentLimit(5)                       // Set current limit to 40A, setting to 5 for testing.
            .voltageCompensation(8);                // limits the maximum voltage to the motor to 8V. in this case, it helps with motor and battery performance.

        // Apply configurations
        m_collectorMotor.configure(collectorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        // this.ledManager = ledManager;

        // pieceSensor = new DutyCycleEncoder(0);
        pieceSensor = new DigitalInput(0);
    }
     
    public boolean hasPiece()
    {
        return !pieceSensor.get();
    }

    public void printSensorValue(){
        System.out.println("Raw Value: " + pieceSensor.get());
    }

    public void run(){
        // ledManager.setState(2);   
        m_collectorMotor.set(1);    
    }

    public void run(double factor){
        // ledManager.setState(2);   
        m_collectorMotor.set(1 * factor);    
    }

    public void stop(){
        // ledManager.setState(0);
        m_collectorMotor.set(0.0);
        m_collectorMotor.stopMotor();
    }
    
}