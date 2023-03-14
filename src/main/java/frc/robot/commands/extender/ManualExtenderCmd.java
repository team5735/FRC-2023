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

    public ManualExtenderCmd(ExtenderSubsystem elevatorSubsystem, Supplier<Double> inputSupplier) {
        this.extenderSubsystem = elevatorSubsystem;
        this.inputSupplier = inputSupplier;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double input = this.inputSupplier.get();
        input = (Math.abs(input) > Constants.OIConstants.JOYSTICK_DEADBAND) ? input : 0.0;
        this.extenderSubsystem.setSetpoint(
            Constants.ExtenderConstants.EXTENDER_MAX_LENGTH * input * 0.5
        );
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
