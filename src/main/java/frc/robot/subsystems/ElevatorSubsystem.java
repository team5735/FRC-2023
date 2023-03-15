package frc.robot.subsystems;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  private final DigitalInput bottomHallSensor, topHallSensor;

  private double heightSetpoint;

  public ElevatorSubsystem() {
    this.elevatorLeader = new WPI_TalonFX(Constants.ElevatorConstants.ELEVATOR_LEADER_MOTOR_ID);
    this.elevatorLeader.setInverted(true);
    this.elevatorFollower = new WPI_TalonFX(Constants.ElevatorConstants.ELEVATOR_FOLLOWER_MOTOR_ID);
    this.elevatorFollower.setInverted(true);
    this.elevatorFollower.follow(this.elevatorLeader);

    this.elevatorFeedforward = new ElevatorFeedforward(
        // V = kG + kS*sgn(d) + kV d/dt + kA d/dt^2, sgn = signum which returns the sign
        Constants.ElevatorConstants.ELEVATOR_CHARACTERIZATION_CONSTANTS.getS(),
        Constants.ElevatorConstants.ELEVATOR_CHARACTERIZATION_CONSTANTS.getG(),
        Constants.ElevatorConstants.ELEVATOR_CHARACTERIZATION_CONSTANTS.getV(),
        Constants.ElevatorConstants.ELEVATOR_CHARACTERIZATION_CONSTANTS.getA());

    this.elevatorFeedback = new ProfiledPIDController(
        Constants.ElevatorConstants.ELEVATOR_CHARACTERIZATION_CONSTANTS.getP(),
        Constants.ElevatorConstants.ELEVATOR_CHARACTERIZATION_CONSTANTS.getI(),
        Constants.ElevatorConstants.ELEVATOR_CHARACTERIZATION_CONSTANTS.getD(),
        new TrapezoidProfile.Constraints(0.25, 0.25) // need to tune max vel and accel in m/s and m/s/s
    );

    this.bottomHallSensor = new DigitalInput(Constants.ElevatorConstants.BOTTOM_HALL_SENSOR_ID);
    this.topHallSensor = new DigitalInput(Constants.ElevatorConstants.TOP_HALL_SENSOR_ID);

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
    return Constants.ElevatorConstants.ELEVATORS_METERS_PER_ROTATION
        * UnitConversion.falconToRotations(elevatorLeader.getSelectedSensorPosition());
  }

  /**
   * Get elevator motor velocity in m/s
   */
  private double getElevatorVelocity() {
    return Constants.ElevatorConstants.ELEVATORS_METERS_PER_ROTATION
        * UnitConversion.falconToRotations(elevatorLeader.getSelectedSensorVelocity());
  }

  // TODO: Make the Joystick ending point a setpoint for the motor to stay at
  public void setSetpoint(double heightMeters) {
    if (heightMeters < 0.0 || heightMeters > Constants.ElevatorConstants.HEIGHT_LIMIT) {
      return;
    }

    this.heightSetpoint = heightMeters;
  }
  
  public void setPercentHeight(double percentage) {
    if(percentage <= 1) {
      double height = percentage * Constants.ElevatorConstants.ELEVATOR_TRAVEL_LENGTH;
      setSetpoint(height);
    }
    return;
  }


  @Override
  public void periodic() {
    // WARNING: Semi TESTED

    double voltage = this.elevatorFeedback.calculate(this.getElevatorHeight(), this.heightSetpoint);
    // // Acceleration is 0? could be totally wrong. Want to get to p0rofiled pid
    // controller velocity setpoint
    voltage += this.elevatorFeedforward.calculate(this.elevatorFeedback.getSetpoint().velocity);
    // // Set the voltage
    this.elevatorLeader.setVoltage(voltage);

    SmartDashboard.putNumber("Elevator Setpoint", this.heightSetpoint);
    SmartDashboard.putNumber("Elevator Height", this.getElevatorHeight());
    SmartDashboard.putNumber("Elevator Master (Right) Encoder", elevatorLeader.getSelectedSensorPosition());
    SmartDashboard.putNumber("Elevator Follower (Left) Encoder", elevatorFollower.getSelectedSensorPosition());
  }

  public void manualControl(double input) {
    this.elevatorLeader.setVoltage(12.0 * input);
  }

  public boolean isAtBottom() {
    return this.bottomHallSensor.get();
  }

  public boolean isAtTop() {
    return this.topHallSensor.get();
  }

  public void stopMotors() {
    this.elevatorLeader.stopMotor();
  }
}
