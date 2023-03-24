package frc.robot.commands.swerve;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import edu.wpi.first.wpilibj.SPI;

public class GyroAutocorrectCommand extends CommandBase {
    private final SwerveSubsystem swerveSubsystem;
    private boolean enabled = true; //testing

    public GyroAutocorrectCommand(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        enabled = true;
    }

    @Override
    public void execute() {
        double proportional = - this.swerveSubsystem.getGyroPitchRad() * 1;

        // for safety
        if(proportional > 0.35 || proportional < -0.35) {
            this.swerveSubsystem.brake();
            enabled = false;
            return;
        }

        else if((proportional > 0.072 || proportional < -0.072) && enabled) {
            SwerveModuleState frontLeftState = new SwerveModuleState(proportional, Rotation2d.fromDegrees(0));
            SwerveModuleState frontRightState = new SwerveModuleState(proportional, Rotation2d.fromDegrees(0));
            SwerveModuleState backLeftState = new SwerveModuleState(proportional, Rotation2d.fromDegrees(0));
            SwerveModuleState backRightState = new SwerveModuleState(proportional, Rotation2d.fromDegrees(0));
            SwerveModuleState[] moduleStates = { frontLeftState, frontRightState, backLeftState, backRightState };
            this.swerveSubsystem.setModuleStates(moduleStates, false);
        }
        
        // Additional Safety -- untested
        if(proportional < 0.072 && proportional > -0.072) {
            this.swerveSubsystem.brake();
            enabled = false;
            return;
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // Could trigger brake command from here or do something similar, but it should be fine to use the button command after stopping

        // SwerveModuleState[] moduleStates = { frontLeftState, frontRightState, backLeftState, backRightState };
        // this.swerveSubsystem.setModuleStates(moduleStates);
    }
}