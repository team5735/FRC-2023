package frc.robot.subsystems;

import frc.robot.constants.ExtenderConstants;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ExtenderSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  private final CANSparkMax extenderMotor;
  private final SparkMaxPIDController pidController;
  private RelativeEncoder extenderEncoder;

  public double kP, maxVel, maxAcc;

  public ExtenderSubsystem() {
    this.extenderMotor = new CANSparkMax(ExtenderConstants.EXTEND_MOTOR_ID, MotorType.kBrushless);

    // Initialize pid controller and encoder
    this.pidController = extenderMotor.getPIDController();
    this.extenderEncoder = extenderMotor.getEncoder();

    this.resetMotors();

    // Set pid coefficients
    kP = .1;
    this.pidController.setP(kP);

    // set smart motion values (in rpm)
    maxVel = 1000;
    maxAcc = 500;

    int smartMotionSlot = 0;
    this.pidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    this.pidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);

  }

  private void resetMotors() {
    this.extenderMotor.getEncoder().setPosition(0);
  }
  

  @Override
  public void periodic() {
    
    SmartDashboard.putNumber("current encoder", extenderEncoder.getPosition());
    
    //Get input from smart dashboard (for easy value updates)
    double p = SmartDashboard.getNumber("P val", 0);
    if (p != kP) {
      this.pidController.setP(p);
    }

    double maxV = SmartDashboard.getNumber("max vel", 0);
    double maxA = SmartDashboard.getNumber("max accel", 0);

    if(maxV != maxVel) {
      this.pidController.setSmartMotionMaxVelocity(maxV, 0);
    }
    if(maxA != maxAcc) {
      this.pidController.setSmartMotionMaxAccel(maxA, 0);
    }

    // set setpoint
    double setPoint, currentPosition;
    setPoint = 100;
    //setPoint = SmartDashboard.getNumber("set position", 0);

    // run smart motion stuff
    this.pidController.setReference(setPoint, ControlType.kSmartMotion);
    currentPosition = extenderEncoder.getPosition();

    //put data
    SmartDashboard.putNumber("set point", setPoint);
    SmartDashboard.putNumber("current position", currentPosition);
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
