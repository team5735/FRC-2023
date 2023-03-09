package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class ManualElevatorCmd extends CommandBase {

    private final ElevatorSubsystem elevatorSubsystem;
    private final Supplier<Double> inputSupplier;

    public ManualElevatorCmd(ElevatorSubsystem elevatorSubsystem, Supplier<Double> inputSupplier) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.inputSupplier = inputSupplier;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double input = this.inputSupplier.get() * 0.25;
        input = (Math.abs(input) > Constants.OIConstants.JOYSTICK_DEADBAND) ? input : 0.0;
        this.elevatorSubsystem.manualControl(input);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
