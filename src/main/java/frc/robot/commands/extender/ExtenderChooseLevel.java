// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.extender;

import frc.robot.subsystems.ExtenderSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ExtenderChooseLevel extends CommandBase {

  private final ExtenderSubsystem extenderSubsystem;
  private double goalEncoderValue;

  /** Creates a new ExtenderChooseLevel. */
  public ExtenderChooseLevel(ExtenderSubsystem extenderSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.extenderSubsystem = extenderSubsystem;
    //Set to a default of some sort
    this.goalEncoderValue = 1;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Specifically for the lower level
    goalEncoderValue = extenderSubsystem.getGoalEncoderValue(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
