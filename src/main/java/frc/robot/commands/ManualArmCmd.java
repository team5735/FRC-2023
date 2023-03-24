package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
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
        // hall sensor not in use
        // if (input <= 0 && this.elevatorSubsystem.isAtBottom()) {
        //     return;
        // }
        // if (input >= 0 && this.elevatorSubsystem.isAtTop()) {
        //     return;
        // }
        // if (input > 0) {
        //     this.elevatorSubsystem.setSetpoint(
        //         this.elevatorSubsystem.getSetpoint() + 0.07
        //     );
        // } else {
        //     this.elevatorSubsystem.setSetpoint(
        //         this.elevatorSubsystem.getSetpoint() - 0.07
        //     );
        // }
        // if (input > 0 || input < 0) {
        //     elevatorSubsystem.setSetpoint(elevatorSubsystem.getSetpoint() + 0.1 * input);
        // }
        // this.elevatorSubsystem.setSetpoint(
        //     Constants.ElevatorConstants.HEIGHT_LIMIT * input * 0.5
        // );
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
