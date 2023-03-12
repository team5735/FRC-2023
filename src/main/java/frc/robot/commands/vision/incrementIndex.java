package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.vision.VisionSubsystem;

public class incrementIndex extends CommandBase {
    private final VisionSubsystem vision;

    public incrementIndex(VisionSubsystem inVision) {
        vision = inVision;
        addRequirements(vision);
    }

    @Override
    public void initialize() {
        vision.pipelineRight();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        return;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
