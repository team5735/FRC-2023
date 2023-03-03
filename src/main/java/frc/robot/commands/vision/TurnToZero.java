package frc.robot.commands.vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.VisionConstants;

public class TurnToZero extends CommandBase {
    private SwerveSubsystem drivetrain;
    private VisionSubsystem vision;
    private boolean isFinished;
    private long startTime;

    //TODO(malish): Calibrate PID!!
    private PIDController pid = new PIDController(0, 0, 0);

    public TurnToZero(VisionSubsystem v, SwerveSubsystem s) {
        vision = v;
        drivetrain = s;
        addRequirements(v);
        addRequirements(s);
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
        pid.reset();
    }

    @Override
    public void execute() {
        isFinished = false;

        if (System.currentTimeMillis() - startTime > VisionConstants.TURN_TIMEOUT) {
            isFinished = true;
            return;
        }

        
    }
}
