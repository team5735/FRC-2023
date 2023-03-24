package frc.robot.commands.arm;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class ManualArmCmd extends CommandBase {

    private final ArmSubsystem armSubsystem;
    private final Supplier<Double> inputSupplier;

    public ManualArmCmd(ArmSubsystem armSubsystem, Supplier<Double> inputSupplier) {
        this.armSubsystem = armSubsystem;
        this.inputSupplier = inputSupplier;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        this.armSubsystem.resetMotors();
    }

    @Override
    public void execute() {
        double input = this.inputSupplier.get();
        input = (Math.abs(input) > Constants.OIConstants.JOYSTICK_DEADBAND) ? input : 0.0;
        this.armSubsystem.manualControl(Math.abs(input));
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
