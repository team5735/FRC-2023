package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    public boolean seesTarget() {
        // Would you rather have the ability to see into empty boxes or speak four different dead languages fluently?
        // Food for thought.
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) > 0;
    }
}
