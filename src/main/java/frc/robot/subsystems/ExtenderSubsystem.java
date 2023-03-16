package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.constants.ExtenderConstants;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ExtenderSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  private final CANSparkMax extenderMotor;
  private final ElevatorFeedforward extenderFeedForward;
  private final ProfiledPIDController extenderFeedback;

  private double extenderSetpoint; // where you want the extender to be, in meters

  public ExtenderSubsystem() {
    this.extenderMotor = new CANSparkMax(ExtenderConstants.EXTEND_MOTOR_ID, MotorType.kBrushless);
    this.extenderFeedForward = new ElevatorFeedforward(
        // V = kG + kS*sgn(d) + kV d/dt + kA d/dt^2, sgn = signum which returns the sign
        0, 1, 0, 0);

    // TODO: Find constants that work. Start with a small P value, velocity, and
    // acceleration.
    this.extenderFeedback = new ProfiledPIDController(
        7, // P value
        0.0, // I vaue
        0.0, // D value
        new TrapezoidProfile.Constraints(
            0.15, // max velocity m/s
            0.25 // max acceleration m/s/s
        ));

    SmartDashboard.putString("extend", "created");

    this.resetMotors();
  }

  private void resetMotors() {
    this.extenderMotor.getEncoder().setPosition(0);
  }

  /**
   * Gets how far the extender has extended, in meters
   */
  public double getExtenderPosition() {
    return this.extenderMotor.getEncoder()
        .getPosition() // in rotations
        * 0.14061; // meters / rotation
    // 5.25 inches per 1 motor rotation
  }

  /**
   * Move the extender out to X meters
   */
  public void setSetpoint(double setpointMeters) {
    if (setpointMeters < 0.0
        || setpointMeters > Constants.ExtenderConstants.EXTENDER_MAX_LENGTH) {
      return;
    }
    this.extenderSetpoint = setpointMeters;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run (every 20ms default)

    if (this.getExtenderPosition() >= Constants.ExtenderConstants.EXTENDER_MAX_LENGTH - 0.05) {
      return;
    }

    // WARNING: UNTESTED
    double voltage = this.extenderFeedback.calculate(this.getExtenderPosition(), this.extenderSetpoint);
    voltage += this.extenderFeedForward.calculate(this.extenderFeedback.getSetpoint().velocity);
    // Set the voltage
    this.extenderMotor.setVoltage(voltage);

    // this.extenderMotor.set(this.pidController.calculate(this.getExtenderPosition(),
    // this.extenderSetpoint));

    SmartDashboard.putNumber("Extender Setpoint", this.extenderSetpoint);
    SmartDashboard.putNumber("Extender Position (m)", this.getExtenderPosition());
    SmartDashboard.putNumber("Extender Motor Encoder", this.extenderMotor.getEncoder().getPosition());
  }

  // Bella's code

  public void extenderMove(double speed) {
    extenderMotor.set(speed);
  }

  public void extenderStop() {
    extenderMotor.set(0);
  }

  public double getCurrentEncoderPosition() {
    return extenderMotor.getEncoder().getPosition();
  }

  // Based on desired level and current encoder value, figure out what encoder
  // value need to reach
  public double setGoalEncoderPosition(int desiredLevel) {
    desiredLevel--;
    return ExtenderConstants.LEVEL_ENCODER_VALS[desiredLevel];
  }

}
