// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.GrabberSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class GrabberCommand extends CommandBase {

  public enum GrabberDirection {
    IN, OUT
  }

  private final GrabberSubsystem grabberSubsystem;
  private final GrabberDirection direction;

  /** Creates a new GrabberCommand. */
  public GrabberCommand(GrabberSubsystem grabberSubsystem, GrabberDirection direction) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.grabberSubsystem = grabberSubsystem;
    this.direction = direction;

    addRequirements(grabberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (direction == GrabberDirection.IN) {
      this.grabberSubsystem.grabberIn();
    } else {
      this.grabberSubsystem.grabberOut();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.grabberSubsystem.grabberStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
