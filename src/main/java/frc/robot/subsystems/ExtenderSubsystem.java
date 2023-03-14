package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.constants.ExtenderConstants;

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
  private final ProfiledPIDController pidController;

  private double extenderSetpoint; // where you want the extender to be, in meters

  public ExtenderSubsystem() {
    this.extenderMotor = new CANSparkMax(ExtenderConstants.EXTEND_MOTOR_ID, MotorType.kBrushless);

    // TODO: Find constants that work. Start with a small P value, velocity, and
    // acceleration.
    this.pidController = new ProfiledPIDController(
        0.01, // P value
        0.0, // I vaue
        0.0, // D value
        new TrapezoidProfile.Constraints(
            0.05, // max velocity m/s
            0.05 // max acceleration m/s/s
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
        * Units.inchesToMeters(5.25);
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

    // WARNING: UNTESTED
    double voltage = this.pidController.calculate(
        this.getExtenderPosition(), // where the extender is right now
        this.extenderSetpoint // where you want the extender to be
    );
    // Set the voltage
    this.extenderMotor.setVoltage(voltage);

    // this.extenderMotor.set(this.pidController.calculate(this.getExtenderPosition(),
    // this.extenderSetpoint));

    SmartDashboard.putNumber("Extender Setpoint", this.extenderSetpoint);
    SmartDashboard.putNumber("Extender Position (m)", this.getExtenderPosition());
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
