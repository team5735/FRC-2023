package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.Constants.ExtenderConstants;

public class AutoMoveExtenderCommand extends CommandBase {

    private final ExtenderSubsystem extenderSubsystem;
    private final int level;

    public AutoMoveExtenderCommand(ExtenderSubsystem extenderSubsystem, int level) {
        this.extenderSubsystem = extenderSubsystem;
        this.level = level;
        addRequirements(extenderSubsystem);
    }

    @Override
    public void initialize() {
        // Untested
        // Could move to constants folder
        // Zero
        if(level == 0) {
            extenderSubsystem.setSetpoint(0);
        }
        // Low
        else if(level == 1) {
            extenderSubsystem.setSetpoint(0.2);
        }
        // Mid
        else if(level == 2) {
            extenderSubsystem.setSetpoint(0.65);
        }
        // High
        else if(level == 3) {
            extenderSubsystem.setSetpoint(ExtenderConstants.EXTENDER_MAX_LENGTH - 0.1);
        }
        else{
            return;
        }
    }

    @Override
    public void execute() {
        //should likely include limit switch if used
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
