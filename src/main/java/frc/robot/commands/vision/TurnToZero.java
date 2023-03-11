package frc.robot.commands.vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.Constants.VisionConstants;

public class TurnToZero extends CommandBase {
    private SwerveSubsystem drivetrain;
    private VisionSubsystem vision;
    private boolean isFinished;
    private long startTime;

    // private static final SwerveModuleState[] DONT_SEE_STATES = {

    // };

    // TODO: Calibrate PID. These values are arbitrary and I don't know if they're
    // good.
    private PIDController pid = new PIDController(0.0727, 0, 0);

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

        // check for team target (important, don't want to be lining up with the
        // opponents' target)
        // ChassisSpeed uses CCW rotation.
        if (!vision.seesTeamTarget()) {
            // maybe too fast?
            ChassisSpeeds turn = new ChassisSpeeds(0, 0, Units.degreesToRadians(90));
            //this.drivetrain.setChassisSpeed(turn);
            drivetrain.openLoopDrive(turn);
            return;
        }

        else {
            // it's in degrees if I recall right, but I'm not 100% on that
            double desired = pid.calculate(vision.getYaw(), 0);
            ChassisSpeeds turn = new ChassisSpeeds(0, 0, Units.degreesToRadians(desired));
            //this.drivetrain.setChassisSpeed(turn);
            drivetrain.openLoopDrive(turn);
        }
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        // System.out.println("Turn to target over");
    }
}
