package frc.robot.subsystems;

import frc.robot.constants.ExtenderConstants;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ExtenderSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  private final CANSparkMax extenderMotor;



  public ExtenderSubsystem() {
    // Basic framework, unknown if this setup is correct
    // could create a new constant file, not worth the trouble
    this.extenderMotor = new CANSparkMax(ExtenderConstants.EXTEND_MOTOR_ID, MotorType.kBrushless);

    SmartDashboard.putString("extend", "created");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run (every 20ms default)
    SmartDashboard.putNumber("Current motor speed", extenderMotor.get());
  }

  public void extenderMove(double speed) {
    extenderMotor.set(speed);
  }

  public void extenderStop() {
    extenderMotor.set(0);
  }

  public double getCurrentEncoderPosition() {
    return extenderMotor.getEncoder().getPosition();
  }

  //Based on desired level and current encoder value, figure out what encoder value need to reach
  public double setGoalEncoderPosition(int desiredLevel) {
    desiredLevel--;
    return ExtenderConstants.LEVEL_ENCODER_VALS[desiredLevel];
  }

}
