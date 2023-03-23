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
  private final ArmFeedforward leftFeedforward, rightFeedforward;
  private final ProfiledPIDController leftFeedback, rightFeedback;

  private double heightSetpoint;

  public ArmSubsystem() {
    this.armLeft = new WPI_TalonFX(Constants.ArmConstants.ARM_LEFT_MOTOR_ID);
    this.armLeft.setInverted(true);
    this.armRight = new WPI_TalonFX(Constants.ArmConstants.ARM_RIGHT_MOTOR_ID);
    this.armRight.setInverted(true);
    // this.elevatorFollower.follow(this.elevatorLeader);

    this.leftFeedforward = new ArmFeedforward(
        // V = kG + kS*sgn(d) + kV d/dt + kA d/dt^2, sgn = signum which returns the sign
        Constants.ArmConstants.ARM_CHARACTERIZATION_CONSTANTS.getS(),
        Constants.ArmConstants.ARM_CHARACTERIZATION_CONSTANTS.getG(),
        Constants.ArmConstants.ARM_CHARACTERIZATION_CONSTANTS.getV(),
        Constants.ArmConstants.ARM_CHARACTERIZATION_CONSTANTS.getA());

    this.rightFeedforward = new ArmFeedforward(
        // V = kG + kS*sgn(d) + kV d/dt + kA d/dt^2, sgn = signum which returns the sign
        Constants.ArmConstants.ARM_CHARACTERIZATION_CONSTANTS.getS(),
        Constants.ArmConstants.ARM_CHARACTERIZATION_CONSTANTS.getG(),
        Constants.ArmConstants.ARM_CHARACTERIZATION_CONSTANTS.getV(),
        Constants.ArmConstants.ARM_CHARACTERIZATION_CONSTANTS.getA());

    this.leftFeedback = new ProfiledPIDController(
        Constants.ArmConstants.ARM_CHARACTERIZATION_CONSTANTS.getP(),
        Constants.ArmConstants.ARM_CHARACTERIZATION_CONSTANTS.getI(),
        Constants.ArmConstants.ARM_CHARACTERIZATION_CONSTANTS.getD(),
        new TrapezoidProfile.Constraints(0.25, 0.25) // need to tune max vel and accel in m/s and m/s/s
    );

    this.rightFeedback = new ProfiledPIDController(
        Constants.ArmConstants.ARM_CHARACTERIZATION_CONSTANTS.getP() + 0.025, // Compensate for sag?
        Constants.ArmConstants.ARM_CHARACTERIZATION_CONSTANTS.getI(),
        Constants.ArmConstants.ARM_CHARACTERIZATION_CONSTANTS.getD(),
        new TrapezoidProfile.Constraints(0.25, 0.25) 
        // need to tune max vel and accel in m/s and m/s^2 -- Could be why the elevator wasn't super fast
    );

    this.resetMotors();
  }

  public void resetMotors() {
    this.armLeft.setSelectedSensorPosition(0);
    this.armRight.setSelectedSensorPosition(0);
  }

  /**
   * Get the current elevator height in meters
   */
  public double getArmCurrentRotations() {
    return (this.getLeftMotorRotations() + this.getRightMotorRotations()) / 2.0;
  }

  public double getLeftMotorRotations() {
    return Constants.ArmConstants.ARM_RADIANS_PER_ROTATION
        * UnitConversion.falconToRotations(armLeft.getSelectedSensorPosition());
  }

  public double getRightMotorRotations() {
    return Constants.ArmConstants.ARM_RADIANS_PER_ROTATION
        * UnitConversion.falconToRotations(armRight.getSelectedSensorPosition());
  }

  /**
   * Get elevator motor velocity in m/s
   */
//   private double getElevatorVelocity() {
//     return Constants.ArmConstants.ARM_RADIANS_PER_ROTATION
//         * UnitConversion.falconToRotations(armLeft.getSelectedSensorVelocity());
//   }

  // TODO: Make the Joystick ending point a setpoint for the motor to stay at
  // TODO: Make sure that this doesn't cancel when the joystick is pressed high
  public void setSetpoint(double heightMeters) {
    if (heightMeters < 0.0) {
      this.heightSetpoint = 0.0;
      return;
    }

    this.heightSetpoint = heightMeters;

    if (heightMeters > Constants.ElevatorConstants.HEIGHT_LIMIT) {
      this.heightSetpoint = Constants.ElevatorConstants.HEIGHT_LIMIT;
      return;
    }
  }

  public double getSetpoint() {
    return this.heightSetpoint;
  }

  public void setPercentHeight(double percentage) {
    if (percentage <= 1) {
      double height = percentage * Constants.ArmConstants.ARM_ROTATION_LIMIT;
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
      setSetpoint(0.4); // ? -- To Test
    }
    // Mid
    else if (level == 2) {
      setSetpoint(ArmConstants.ARM_ROTATION_LIMIT);
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

    double leftVoltage = this.leftFeedback.calculate(this.getLeftMotorRotations(), this.heightSetpoint);
    leftVoltage += this.leftFeedforward.calculate(this.getLeftMotorRotations(), this.leftFeedback.getSetpoint().velocity);
    
    double rightVoltage = this.rightFeedback.calculate(this.getRightMotorRotations(), this.heightSetpoint);
    rightVoltage += this.rightFeedforward.calculate(this.getRightMotorRotations(), this.rightFeedback.getSetpoint().velocity);

    // Set the voltage
    this.armLeft.setVoltage(leftVoltage);
    this.armRight.setVoltage(rightVoltage);

    SmartDashboard.putNumber("Arm Setpoint", this.heightSetpoint);
    SmartDashboard.putNumber("Arm Height", this.getArmCurrentRotations());
    SmartDashboard.putNumber("Arm Master (Right) Encoder", armLeft.getSelectedSensorPosition());
    SmartDashboard.putNumber("Arm Follower (Left) Encoder", armRight.getSelectedSensorPosition());
  }

  public void manualControl(double input) {
    this.armLeft.setVoltage(12.0 * input);
  }

  public void stopMotors() {
    this.armLeft.stopMotor();
  }
}
