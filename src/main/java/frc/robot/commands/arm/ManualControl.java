package frc.robot.commands.arm;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.ArmSubsystem;

public class ManualControl extends Command 
{
    ArmSubsystem c_arm;
    DoubleSupplier c_input;

    public ManualControl(ArmSubsystem arm, DoubleSupplier input) {
        c_arm = arm;
        c_input = input;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        c_arm.drive(c_input.getAsDouble() * 2);
        System.out.println("-----------------------------execute");
    }

    public void maintainPosition() {
        c_arm.drive(c_input.getAsDouble() * 2);
    }

}
