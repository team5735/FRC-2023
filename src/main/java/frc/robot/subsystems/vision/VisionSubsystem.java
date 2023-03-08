package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;

// Using limelight 3D support

// limelight knowledge: positive return value in targetpose_robotspace[6]
// means that you need to turn right. negative is left.
public class VisionSubsystem extends SubsystemBase {
    private NetworkTable limelightTable;
    private int pipelineCount;

    public VisionSubsystem(int pipelineCount) {
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        this.pipelineCount = pipelineCount;
    }

    // checks if limelight sees anything
    public boolean seesTarget() {
        // Would you rather have the ability to see into empty boxes or speak four different dead languages fluently?
        // Food for thought.
        return limelightTable.getEntry("tv").getBoolean(false);
    }

    // different teams have different AprilTags, this takes that into account.
    public boolean seesTeamTarget() {
        if (!seesTarget())
            return false;

        DriverStation.Alliance a = DriverStation.getAlliance();
        long tid = getPrimaryTargetID();

        if (a == DriverStation.Alliance.Invalid) {
            return seesTarget();
        }

        // red community: 1, 2, 3
        if (a == DriverStation.Alliance.Red &&  tid > 0 && tid < 4) {
            return true;
        }

        // blue community: 6, 7, 8
        if (a == DriverStation.Alliance.Blue && tid > 5 && tid < 9) {
            return true;
        }

         // community/troll target (possible strategy to consider while people are wasting time in the stands)
        return false;
    }

    // TODO(ari): maybe add seesTeamSubstation to turn to drop thingy

    // public void toggleAnnoyingMode() {
    //     limelightTable.getEntry("ledMode").setNumber(limelightTable.getEntry("ledMode").getNumber(2) == (Number)(2) ? 0 : 2);   
    // }

    public double getPipelineLatency() {
        return limelightTable.getEntry("tl").getDouble(0);
    }

    public long getPrimaryTargetID() {
        return limelightTable.getEntry("tid").getInteger(-1);
    }

    public double[] getOffsetCameraspace() {
        return limelightTable.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);
    }
    public double[] getOffsetTargetspace() {
        return limelightTable.getEntry("camerapose_targetspace").getDoubleArray(new double[6]);
    }

    public long getPipelineIndex() {
        return limelightTable.getEntry("getpipe").getInteger(-1);
    }

    public void pipelineLeft() {
        limelightTable.getEntry("pipeline").setInteger(Math.max(0, getPipelineIndex() - 1));
    }

    public void pipelineRight() {
        limelightTable.getEntry("pipeline").setInteger(Math.min(getPipelineIndex() + 1, pipelineCount - 1));
    }

    public double getYaw() {
        // i'm like 80% sure this is yaw
        return getOffsetCameraspace()[6];
    }
}
