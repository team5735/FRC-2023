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
    private final Supplier<Double> currentElevatorHeight;

    public ManualExtenderCmd(ExtenderSubsystem extenderSubsystem, Supplier<Double> inputSupplier, Supplier<Double> currentElevatorHeight) {
        this.extenderSubsystem = extenderSubsystem;
        this.inputSupplier = inputSupplier;
        this.currentElevatorHeight = currentElevatorHeight;
        addRequirements(extenderSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (this.currentElevatorHeight.get() < Constants.ExtenderConstants.EXTENDER_ELEVATOR_MIN_HEIGHT) {
            return;
        }

        double input = this.inputSupplier.get();
        input = (Math.abs(input) > 0.5) ? input : 0.0; // if trigger not halfway down, don't do anything
        
        if (input > 0) {
            extenderSubsystem.setSetpoint(extenderSubsystem.getSetpoint() + 0.5);
        } else if (input < 0) {
            extenderSubsystem.setSetpoint(extenderSubsystem.getSetpoint() - 0.5);
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
