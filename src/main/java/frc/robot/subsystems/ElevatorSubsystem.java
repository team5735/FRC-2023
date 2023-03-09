package frc.robot.subsystems;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import cfutil.UnitConversion;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  private final WPI_TalonFX elevatorLeader, elevatorFollower;
  private final ElevatorFeedforward elevatorFeedforward;
  private final ProfiledPIDController elevatorFeedback;

  private double heightSetpoint;

  public ElevatorSubsystem() {
    this.elevatorLeader = new WPI_TalonFX(Constants.MotorConstants.ELEVATOR_LEADER_MOTOR_ID);
    this.elevatorLeader.setInverted(true);
    this.elevatorFollower = new WPI_TalonFX(Constants.MotorConstants.ELEVATOR_FOLLOWER_MOTOR_ID);
    this.elevatorFollower.setInverted(false);
    this.elevatorFollower.follow(this.elevatorLeader);

    this.elevatorFeedforward = new ElevatorFeedforward(
        Constants.MotorConstants.ELEVATOR_CHARACTERIZATION_CONSTANTS.getS(),
        Constants.MotorConstants.ELEVATOR_CHARACTERIZATION_CONSTANTS.getG(),
        Constants.MotorConstants.ELEVATOR_CHARACTERIZATION_CONSTANTS.getV(),
        Constants.MotorConstants.ELEVATOR_CHARACTERIZATION_CONSTANTS.getA());

    this.elevatorFeedback = new ProfiledPIDController(
        Constants.MotorConstants.ELEVATOR_CHARACTERIZATION_CONSTANTS.getP(),
        Constants.MotorConstants.ELEVATOR_CHARACTERIZATION_CONSTANTS.getI(),
        Constants.MotorConstants.ELEVATOR_CHARACTERIZATION_CONSTANTS.getD(),
        new TrapezoidProfile.Constraints(0, 0) // need to tune max vel and accel in m/s and m/s/s
    );

    this.resetMotors();
  }

  private void resetMotors() {
    this.elevatorLeader.setSelectedSensorPosition(0);
    this.elevatorFollower.setSelectedSensorPosition(0);
  }

  /**
   * Get the current elevator height in meters
   */
  private double getElevatorHeight() {
    // Must have motor position reset to zero beforehand for this to work well
    return Constants.ElevatorConstants.ELEVATORS_METERS_PER_ROTATION * UnitConversion.falconToRotations(elevatorLeader.getSelectedSensorPosition());
  }

  /**
   * Get elevator motor velocity in m/s
   */
  private double getElevatorVelocity() {
    return Constants.ElevatorConstants.ELEVATORS_METERS_PER_ROTATION * UnitConversion.falconToRotations(elevatorLeader.getSelectedSensorVelocity());
  }

  public void setSetpoint(double heightMeters) {
    if (heightMeters < 0.0 || heightMeters > Constants.ElevatorConstants.HEIGHT_LIMIT) {
      return;
    }

    this.heightSetpoint = heightMeters;
  }

  @Override
  public void periodic() {
    // WARNING: UNTESTED

    // double voltage = this.elevatorFeedback.calculate(this.getElevatorHeight(), this.heightSetpoint);
    // // Acceleration is 0? could be totally wrong. Want to get to profiled pid controller velocity setpoint
    // voltage += this.elevatorFeedforward.calculate(this.elevatorFeedback.getSetpoint().velocity);
    // // Set the voltage
    // this.elevatorLeader.setVoltage(voltage);
  }

  public void manualControl(double input) {
    this.elevatorLeader.setVoltage(12.0 * input);
  }

  // public void eleavtorStop() {
  // elevatorLeft.stopMotor();
  // elevatorRight.stopMotor();
  // }
}
