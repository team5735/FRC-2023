package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class ExtenderSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  private final WPI_TalonFX extenderController;


  public ExtenderSubsystem() {
    // Basic framework, unknown if this setup is correct, needs different device numbers
    // could create a new constant file, but maybe not worth the trouble
    this.extenderController = new WPI_TalonFX(102);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run (every 20ms default)
  }
}
