package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class MoveStraightCmd extends CommandBase {

    public enum MoveDirection {
        FORWARD, BACKWARD
    }

    private final SwerveSubsystem swerveSubsystem;
    private final MoveDirection moveDirection;

    public MoveStraightCmd(SwerveSubsystem swerveSubsystem, MoveDirection moveDirection) {
        this.swerveSubsystem = swerveSubsystem;
        this.moveDirection = moveDirection;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double vx = 0.25;
        if (this.moveDirection == MoveDirection.BACKWARD) {
            vx *= -1;
        }
        this.swerveSubsystem.openLoopDrive(
            new ChassisSpeeds(vx, 0, 0)
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        this.swerveSubsystem.stopModules();
    }
}