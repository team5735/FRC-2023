package frc.robot.subsystems;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import cfutil.UnitConversion;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  private final WPI_TalonFX elevatorLeader, elevatorFollower;
  private final ElevatorFeedforward elevatorFeedforward;
  private final ProfiledPIDController elevatorFeedback;
  private final double inchesPerRotation, metersPerRotation;

  private double heightSetpoint;

  public ElevatorSubsystem() {
    this.elevatorLeader = new WPI_TalonFX(Constants.MotorConstants.ELEVATOR_LEADER_MOTOR_ID);
    this.elevatorFollower = new WPI_TalonFX(Constants.MotorConstants.ELEVATOR_FOLLOWER_MOTOR_ID);
    // Is the motor inverted? Based on how you set elevatorRight to negative, I'm
    // gonna assume so.
    this.elevatorFollower.setInverted(true);
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

    // Gear Ratio * Sprocket Teeth # * Length per chain link
    this.inchesPerRotation = 
        Constants.ElevatorConstants.gearRatio *
        Constants.ElevatorConstants.sprocketTeethNumber *
        Constants.ElevatorConstants.chainLinkLength;
    this.metersPerRotation = Units.inchesToMeters(inchesPerRotation);

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
    double elevatorHeight = metersPerRotation * UnitConversion.falconToRotations(elevatorLeader.getSelectedSensorPosition());
    return elevatorHeight;
  }

  /**
   * Get elevator motor velocity in m/s
   */
  private double getElevatorVelocity() {
    double elevatorVelocity = metersPerRotation * UnitConversion.falconToRotations(elevatorLeader.getSelectedSensorVelocity());
    return elevatorVelocity;
  }

  public void setSetpoint(double heightMeters) {
    if (heightMeters < 0.0 || heightMeters > Constants.ElevatorConstants.HEIGHT_LIMIT) {
      return;
    }

    this.heightSetpoint = heightMeters;
  }

  @Override
  public void periodic() {
    double voltage = this.elevatorFeedback.calculate(this.getElevatorHeight(), this.heightSetpoint);
    // Acceleration is 0? could be totally wrong. Want to get to profiled pid controller velocity setpoint
    voltage += this.elevatorFeedforward.calculate(this.elevatorFeedback.getSetpoint().velocity);
    // Set the voltage
    this.elevatorLeader.setVoltage(voltage);
  }

  // public void elevatorControl(double joystickInput) {
  // elevatorLeft.set(joystickInput);
  // elevatorRight.set(-joystickInput);

  // }

  // public void eleavtorStop() {
  // elevatorLeft.stopMotor();
  // elevatorRight.stopMotor();
  // }
}
