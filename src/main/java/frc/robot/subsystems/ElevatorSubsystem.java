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

  private final WPI_TalonFX elevatorLeft, elevatorRight;
  private final ElevatorFeedforward leftFeedforward, rightFeedforward;
  private final ProfiledPIDController leftFeedback, rightFeedback;

  private final DigitalInput bottomHallSensor, topHallSensor;

  private double heightSetpoint;

  public ElevatorSubsystem() {
    this.elevatorLeft = new WPI_TalonFX(Constants.ElevatorConstants.ELEVATOR_LEFT_MOTOR_ID);
    this.elevatorLeft.setInverted(true);
    this.elevatorRight = new WPI_TalonFX(Constants.ElevatorConstants.ELEVATOR_RIGHT_MOTOR_ID);
    this.elevatorRight.setInverted(true);
    // this.elevatorFollower.follow(this.elevatorLeader);

    this.leftFeedforward = new ElevatorFeedforward(
        // V = kG + kS*sgn(d) + kV d/dt + kA d/dt^2, sgn = signum which returns the sign
        Constants.ElevatorConstants.ELEVATOR_CHARACTERIZATION_CONSTANTS.getS(),
        Constants.ElevatorConstants.ELEVATOR_CHARACTERIZATION_CONSTANTS.getG(),
        Constants.ElevatorConstants.ELEVATOR_CHARACTERIZATION_CONSTANTS.getV(),
        Constants.ElevatorConstants.ELEVATOR_CHARACTERIZATION_CONSTANTS.getA());

    this.rightFeedforward = new ElevatorFeedforward(
        // V = kG + kS*sgn(d) + kV d/dt + kA d/dt^2, sgn = signum which returns the sign
        Constants.ElevatorConstants.ELEVATOR_CHARACTERIZATION_CONSTANTS.getS(),
        Constants.ElevatorConstants.ELEVATOR_CHARACTERIZATION_CONSTANTS.getG(),
        Constants.ElevatorConstants.ELEVATOR_CHARACTERIZATION_CONSTANTS.getV(),
        Constants.ElevatorConstants.ELEVATOR_CHARACTERIZATION_CONSTANTS.getA());

    this.leftFeedback = new ProfiledPIDController(
        Constants.ElevatorConstants.ELEVATOR_CHARACTERIZATION_CONSTANTS.getP(),
        Constants.ElevatorConstants.ELEVATOR_CHARACTERIZATION_CONSTANTS.getI(),
        Constants.ElevatorConstants.ELEVATOR_CHARACTERIZATION_CONSTANTS.getD(),
        new TrapezoidProfile.Constraints(0.25, 0.25) // need to tune max vel and accel in m/s and m/s/s
    );

    this.rightFeedback = new ProfiledPIDController(
      Constants.ElevatorConstants.ELEVATOR_CHARACTERIZATION_CONSTANTS.getP() + 0.1, // compensate sag
      Constants.ElevatorConstants.ELEVATOR_CHARACTERIZATION_CONSTANTS.getI(),
      Constants.ElevatorConstants.ELEVATOR_CHARACTERIZATION_CONSTANTS.getD(),
        new TrapezoidProfile.Constraints(0.25, 0.25) // need to tune max vel and accel in m/s and m/s/s
    );

    this.bottomHallSensor = new DigitalInput(Constants.ElevatorConstants.BOTTOM_HALL_SENSOR_ID);
    this.topHallSensor = new DigitalInput(Constants.ElevatorConstants.TOP_HALL_SENSOR_ID);

    this.resetMotors();
  }

  public void resetMotors() {
    this.elevatorLeft.setSelectedSensorPosition(0);
    this.elevatorRight.setSelectedSensorPosition(0);
  }

  /**
   * Get the current elevator height in meters
   */
  public double getElevatorHeight() {
    return (this.getLeftMotorHeight() + this.getRightMotorHeight()) / 2.0;
  }

  public double getLeftMotorHeight() {
    return Constants.ElevatorConstants.ELEVATORS_METERS_PER_ROTATION
        * UnitConversion.falconToRotations(elevatorLeft.getSelectedSensorPosition());
  }

  public double getRightMotorHeight() {
    return Constants.ElevatorConstants.ELEVATORS_METERS_PER_ROTATION
        * UnitConversion.falconToRotations(elevatorRight.getSelectedSensorPosition());
  }

  /**
   * Get elevator motor velocity in m/s
   */
  private double getElevatorVelocity() {
    return Constants.ElevatorConstants.ELEVATORS_METERS_PER_ROTATION
        * UnitConversion.falconToRotations(elevatorLeft.getSelectedSensorVelocity());
  }

  // TODO: Make the Joystick ending point a setpoint for the motor to stay at
  // TODO: Make sure that this doesn't cancel when the joystick is pressed high
  public void setSetpoint(double heightMeters) {
    if (heightMeters < 0.0) {
      this.heightSetpoint = 0.0;
      return;
    }

    if (heightMeters > Constants.ElevatorConstants.HEIGHT_LIMIT) {
      this.heightSetpoint = Constants.ElevatorConstants.HEIGHT_LIMIT;
      return;
    }

    this.heightSetpoint = heightMeters;
  }

  public double getSetpoint() {
    return this.heightSetpoint;
  }

  public void setPercentHeight(double percentage) {
    if (percentage <= 1) {
      double height = percentage * Constants.ElevatorConstants.ELEVATOR_TRAVEL_LENGTH;
      setSetpoint(height);
    }
    return;
  }

  public void setLevel(int level) {

    if (level == 0) {
      setSetpoint(0);
    }
    // Low
    else if (level == 1) {
      setSetpoint(0.4);
    }
    // Mid
    else if (level == 2) {
      setSetpoint(0.65);
    }
    // High
    // else if(level == 3) {
    // elevatorSubsystem.setSetpoint(0);
    // }
    else {
      return;
    }
  }

  @Override
  public void periodic() {
    // WARNING: Semi TESTED

    double leftVoltage = this.leftFeedback.calculate(this.getLeftMotorHeight(), this.heightSetpoint);
    leftVoltage += this.leftFeedforward.calculate(this.leftFeedback.getSetpoint().velocity);
    
    double rightVoltage = this.rightFeedback.calculate(this.getRightMotorHeight(), this.heightSetpoint);
    rightVoltage += this.rightFeedforward.calculate(this.rightFeedback.getSetpoint().velocity);

    // Set the voltage
    this.elevatorLeft.setVoltage(leftVoltage);
    this.elevatorRight.setVoltage(rightVoltage);

    SmartDashboard.putNumber("Elevator Setpoint", this.heightSetpoint);
    SmartDashboard.putNumber("Elevator Height", this.getElevatorHeight());
    SmartDashboard.putNumber("Elevator Master (Right) Encoder", elevatorLeft.getSelectedSensorPosition());
    SmartDashboard.putNumber("Elevator Follower (Left) Encoder", elevatorRight.getSelectedSensorPosition());
  }

  public void manualControl(double input) {
    this.elevatorLeft.setVoltage(12.0 * input);
  }

  public boolean isAtBottom() {
    return this.bottomHallSensor.get();
  }

  public boolean isAtTop() {
    return this.topHallSensor.get();
  }

  public void stopMotors() {
    this.elevatorLeft.stopMotor();
  }
}
