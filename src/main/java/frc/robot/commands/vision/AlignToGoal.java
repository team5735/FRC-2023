package frc.robot.commands.vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.VisionConstants;

// MINGLE IDEA FOR SHORT DISTANCE
public class AlignToGoal extends CommandBase {
    private SwerveSubsystem drivetrain;
    private VisionSubsystem vision;
    private boolean isFinished;
    private long startTime;


    //TODO(malish): Tune PID
    private PIDController pidX = new PIDController(0, 0, 0);
    private PIDController pidY = new PIDController(0, 0, 0);
    public AlignToGoal(VisionSubsystem v, SwerveSubsystem s) {
        vision = v;
        drivetrain = s;
        addRequirements(v);
        addRequirements(s);
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
        pidX.reset();
        pidY.reset();
    }

    @Override
    public void execute() {
        if (System.currentTimeMillis() > startTime + 5000) {
            isFinished = true;
            return;
        }
        // Cancel if it doesn't see a team target
        if (!vision.seesTeamTarget()){
            isFinished = true;
            return;
        }

        // straighten first
        if (vision.getYaw() > 4.0) {
            System.out.println("Straighten the robot first");
            isFinished = true;
            return;
        }

        double[] reallyCoolNumbers = vision.getOffsetCameraspace();
        var x = reallyCoolNumbers[0];
        var y = reallyCoolNumbers[2];
        
        // I solved control theory but apparently the code was "already tried and bad." George orwell predicted this    
        //worse solution:
        var desx = pidX.calculate(x, 0);
        var desy = pidY.calculate(y, 0);
        drivetrain.setChassisSpeed(new ChassisSpeeds(desx, desy, 0));
    }

    @Override
    public boolean isFinished() {
        return isFinished;    
    }
}
