package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class turnToTarget extends CommandBase {
    private SwerveSubsystem drivetrain;
    private VisionSubsystem vision;
    private boolean isFinished;
    private long startTime;

    // maybe implement later TBH
}
