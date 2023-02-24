package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Using limelight 3D support
public class VisionSubsystem extends SubsystemBase {
    private NetworkTable limelightTable;
    private int pipelineCount;
    public VisionSubsystem(int pipelineCount) {
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        this.pipelineCount = pipelineCount;
    }
    public boolean seesTarget() {
        // Would you rather have the ability to see into empty boxes or speak four different dead languages fluently?
        // Food for thought.
        return limelightTable.getEntry("tv").getBoolean(false);
    }

    public void toggleAnnoyingMode() {
        limelightTable.getEntry("ledMode").setNumber(limelightTable.getEntry("ledMode").getNumber(2) == (Number)(2) ? 0 : 2);   
    }

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

}
