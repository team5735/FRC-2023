package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;

@FunctionalInterface
public interface IAutoCommand<SwerveSubsystem, ArmSubsystem, GrabberSubsystem, IntakeSubsystem, Map> {
    public Command apply(SwerveSubsystem swerve, ArmSubsystem arm, GrabberSubsystem grabber, IntakeSubsystem intake, Map eventMap);
}