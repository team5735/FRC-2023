// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.GrabberConstants;

public class GrabberSubsystem extends SubsystemBase {

  private CANSparkMax grabberMotor;
  /** Creates a new GrabberSubsystem. */
  public GrabberSubsystem() {
    grabberMotor = new CANSparkMax(GrabberConstants.GRABBER_MOTOR_ID, MotorType.kBrushless);
    grabberMotor.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void grabberSlowIn() {
    grabberMotor.set(GrabberConstants.GRABBER_SLOW_SPEED);
  }

  public void grabberSlowOut() {
    grabberMotor.set(-GrabberConstants.GRABBER_SLOW_SPEED);
  }

  public void grabberIn() {
    grabberMotor.set(GrabberConstants.GRABBER_IN_SPEED);
  }

  public void grabberOut() {
    grabberMotor.set(GrabberConstants.GRABBER_OUT_SPEED);
  }

  public void grabberStop() {
    grabberMotor.set(0);
  }
}
