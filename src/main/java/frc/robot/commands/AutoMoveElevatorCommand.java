package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;

public class AutoMoveElevatorCommand extends CommandBase {

    private final ElevatorSubsystem elevatorSubsystem;
    private final int level;

    public AutoMoveElevatorCommand(ElevatorSubsystem elevatorSubsystem, int level) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.level = level;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        // Untested
        // Could move to constants folder
        // Zero
        if(level == 0) {
            elevatorSubsystem.setPercentHeight(0);
        }
        // Low
        else if(level == 1) {
            elevatorSubsystem.setPercentHeight(0.1);
        }
        // Mid
        else if(level == 2) {
            elevatorSubsystem.setPercentHeight(0.5);
        }
        // High
        else if(level == 3) {
            elevatorSubsystem.setPercentHeight(0.9);
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
