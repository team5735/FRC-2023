package frc.robot.commands.extender;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class ManualExtenderCmd extends CommandBase {

    private final ExtenderSubsystem extenderSubsystem;
    private final Supplier<Double> inputSupplier;

    public ManualExtenderCmd(ExtenderSubsystem extenderSubsystem, Supplier<Double> inputSupplier) {
        this.extenderSubsystem = extenderSubsystem;
        this.inputSupplier = inputSupplier;
        addRequirements(extenderSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double input = this.inputSupplier.get();
        input = (Math.abs(input) > Constants.OIConstants.JOYSTICK_DEADBAND) ? input : 0.0;
        if (input > 0 || input < 0) {
            extenderSubsystem.setSetpoint(extenderSubsystem.getSetpoint() + 0.1 * input);
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
