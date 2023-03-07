package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  private final WPI_TalonFX elevatorLeft, elevatorRight;
  private final PIDController elevatorLeftPID, elevatorRightPID;



  public ElevatorSubsystem() {
    // Basic framework, unknown if this setup is correct, needs different device numbers
    // could create a new constant file, but maybe not worth the trouble
    this.elevatorLeft = new WPI_TalonFX(55);
    this.elevatorRight = new WPI_TalonFX(57);

    this.elevatorLeftPID = new PIDController(0, 0, 0);
    this.elevatorRightPID = new PIDController(0, 0, 0);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run (every 20ms default)
  }

  public void elevatorControl(double joystickInput) {
    elevatorLeft.set(joystickInput);
    elevatorRight.set(-joystickInput);

  }
  
  public void eleavtorStop() {
    elevatorLeft.stopMotor();
    elevatorRight.stopMotor();
  }
}
