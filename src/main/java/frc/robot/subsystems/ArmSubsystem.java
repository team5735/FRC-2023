package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import cfutil.UnitConversion;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  private final WPI_TalonFX armLeft, armRight;
  private final ArmFeedforward armFF;
  private final ProfiledPIDController armFeedback;

  private Double angleSetpoint;
  private double lastVelocitySetpoint;

  public ArmSubsystem() {
    this.armLeft = new WPI_TalonFX(Constants.ArmConstants.ARM_LEFT_MOTOR_ID);
    this.armLeft.setInverted(true);
    this.armRight = new WPI_TalonFX(Constants.ArmConstants.ARM_RIGHT_MOTOR_ID);
    this.armRight.setInverted(false);
    // this.armRight.follow(this.armLeft);

    this.armFF = new ArmFeedforward(
        Constants.ArmConstants.ARM_CHARACTERIZATION_CONSTANTS.getS(),
        Constants.ArmConstants.ARM_CHARACTERIZATION_CONSTANTS.getG(),
        Constants.ArmConstants.ARM_CHARACTERIZATION_CONSTANTS.getV(),
        Constants.ArmConstants.ARM_CHARACTERIZATION_CONSTANTS.getA());

    this.armFeedback = new ProfiledPIDController(
        Constants.ArmConstants.ARM_CHARACTERIZATION_CONSTANTS.getP(),
        Constants.ArmConstants.ARM_CHARACTERIZATION_CONSTANTS.getI(),
        Constants.ArmConstants.ARM_CHARACTERIZATION_CONSTANTS.getD(),
        new TrapezoidProfile.Constraints(Math.PI / 4, Math.PI / 4) // Pi/2 rad / s, in 2 second
    );

    this.resetMotors();
  }

  public void resetMotors() {
    this.armLeft.setSelectedSensorPosition(0);
  }

  /**
   * Get current arm left angle in radians
   */
  public double getArmLeftAngle() {
    // 1 rotation of falcon motor
    // * (12 / 84) # rotation of 84 tooth gear; falcon gear has 12 teeth attached to
    // 84 teeth gear
    // * 1 84 tooth and 16 tooth gear are on same axle, same # rotations
    // * (16 / 64) # rotation of 64 tooth gear up top
    // * 2 * Math.PI # rotation -> radians
    return (UnitConversion.falconToRotations(this.armLeft.getSelectedSensorPosition())
        * Constants.ArmConstants.ARM_GEAR_RATIO
        * Constants.ArmConstants.PIVOT_POINT_GEAR_RATIO
        * 2 * Math.PI)
        + Constants.ArmConstants.ARM_POSITION_START_RAD; // when sensor position is 0, the angle is actually -90deg
  }

  /**
   * Angle in radians
   */
  public void setSetpoint(double angleSetpoint) {
    if (angleSetpoint < Constants.ArmConstants.ARM_MIN_ANGLE) {
      this.angleSetpoint = Constants.ArmConstants.ARM_MIN_ANGLE;
      return;
    }

    if (angleSetpoint > Constants.ArmConstants.ARM_MAX_ANGLE) {
      this.angleSetpoint = Constants.ArmConstants.ARM_MAX_ANGLE;
      return;
    }

    this.angleSetpoint = angleSetpoint;
  }

  public double getSetpoint() {
    return this.angleSetpoint;
  }

  public void setLevel(int level) {

    if (level == 0) {
      setSetpoint(Units.degreesToRadians(-90));
    }
    // Low
    // else if (level == 1) {
    // setSetpoint(0.4); // ? -- To Test
    // }
    // Mid
    else if (level == 1) {
      setSetpoint(0);
    } else {
      return;
    }
  }

  public boolean isAtSetpoint() {
    return Math.abs(this.getArmLeftAngle() - this.getSetpoint()) < Constants.ArmConstants.ARM_SETPOINT_THRESHOLD;
  }

  @Override
  public void periodic() {
    // Should fix the weird setpoint jump-to-0 error
    if (this.angleSetpoint == null) {
      return;
    }

    double voltage = this.armFeedback.calculate(this.getArmLeftAngle(), this.angleSetpoint);
    voltage += this.armFF.calculate(this.getArmLeftAngle(), this.armFeedback.getSetpoint().velocity,
        (this.armFeedback.getSetpoint().velocity - this.lastVelocitySetpoint) / 0.02);

    // It should work the same if right was follower of left but somehow it doesn't.
    this.armLeft.setVoltage(voltage);
    this.armRight.setVoltage(voltage);

    // SmartDashboard.putNumber("Arm Curr Setpoint Accel",
    // (this.armFeedback.getSetpoint().velocity - this.lastVelocitySetpoint) /
    // 0.02);
    // SmartDashboard.putNumber("Arm Curr Setpoint Pos",
    // this.armFeedback.getSetpoint().position);
    // SmartDashboard.putNumber("Arm Curr Setpoint Vel",
    // this.armFeedback.getSetpoint().velocity);
    // SmartDashboard.putNumber("Arm Goal", this.angleSetpoint);
    // SmartDashboard.putNumber("Arm Angle", this.getArmLeftAngle());
    // // SmartDashboard.putNumber("Arm Voltage FF+FB", voltage);
    // SmartDashboard.putNumber("Arm Motor L Amps",
    // this.armLeft.getStatorCurrent());
    // SmartDashboard.putNumber("Arm Motor R Amps",
    // this.armRight.getStatorCurrent());
    // SmartDashboard.putNumber("Arm Height", this.getArmCurrentRotations());

    this.lastVelocitySetpoint = this.armFeedback.getSetpoint().velocity;
  }

  public void manualControl(double input) {
    this.armLeft.setVoltage(12.0 * input * 0.25);
    this.armRight.setVoltage(12.0 * input * 0.25);
  }

  public void stopMotors() {
    this.armLeft.stopMotor();
  }
}
