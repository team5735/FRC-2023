// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.constants.IntakeConstants;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  private CANSparkMax intakeMotor;
  private CANSparkMax conveyorMotor;

  public IntakeSubsystem() {
    intakeMotor = new CANSparkMax(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
    conveyorMotor = new CANSparkMax(IntakeConstants.CONVEYOR_MOTOR_ID, MotorType.kBrushless);

    //intakeMotor.setInverted(true);
    //conveyorMotor.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run (every 20ms default)
    //Current checks
    SmartDashboard.putNumber("intake_current", intakeMotor.getOutputCurrent());
    SmartDashboard.putNumber("conveyor_current", conveyorMotor.getOutputCurrent());
    if(intakeMotor.getOutputCurrent() < 15) {
      SmartDashboard.putString("LOW CURRENT", "motor controller "+IntakeConstants.INTAKE_MOTOR_ID+" not providing enough current");
    }

    //Check motor direction
    SmartDashboard.putNumber("intake motor direction", intakeMotor.get());
    SmartDashboard.putNumber("conveyor motor direction", conveyorMotor.get());
  }

  public void intakeForward() {
    intakeMotor.set(IntakeConstants.INTAKE_IN_SPEED);
    conveyorMotor.set(IntakeConstants.CONVEYOR_IN_SPEED);
  }

  public void intakeBackward() {
    intakeMotor.set(IntakeConstants.INTAKE_OUT_SPEED);
    conveyorMotor.set(IntakeConstants.CONVEYOR_OUT_SPEED);
  }

  public void intakeStop() {
    intakeMotor.set(0);
    conveyorMotor.set(0);
  }
}
