// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import frc.robot.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeCommand extends CommandBase {

  public enum IntakeDirection {
    IN, OUT
  }

  private final IntakeSubsystem intakeSubsystem;
  private final IntakeDirection direction;

  public IntakeCommand(IntakeSubsystem intakeSubsystem, IntakeDirection direction) {
    this.intakeSubsystem = intakeSubsystem;
    this.direction = direction;
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (direction == IntakeDirection.IN) {
      this.intakeSubsystem.intakeIn();
    } else {
      this.intakeSubsystem.intakeOut();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.intakeSubsystem.intakeStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
