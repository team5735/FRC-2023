package frc.robot.commands;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class TurnMotorFullSpeedCommand extends CommandBase {
    private final SwerveSubsystem swerveSubsystem;

    public TurnMotorFullSpeedCommand(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        this.swerveSubsystem.turnFullPower();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}