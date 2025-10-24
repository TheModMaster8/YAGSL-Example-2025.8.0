package frc.robot.commands.arm;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.ArmSubsystem;

public class ManualControl extends Command 
{
    ArmSubsystem arm;
    DoubleSupplier input;

    public ManualControl(ArmSubsystem arm, DoubleSupplier input) {
        this.arm = arm;
        this.input = input;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        arm.drive(input.getAsDouble() * 2);
    }
}
