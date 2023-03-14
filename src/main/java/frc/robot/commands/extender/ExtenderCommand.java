// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.extender;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtenderSubsystem;

public class ExtenderCommand extends CommandBase {
  /** Creates a new ExtenderCommand. */

  private final ExtenderSubsystem extenderSubsystem;
  private final boolean extendOutwards;

  public ExtenderCommand(ExtenderSubsystem extenderSubsystem, boolean extendOutwards) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.extenderSubsystem = extenderSubsystem;
    this.extendOutwards = extendOutwards;

    addRequirements(extenderSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (this.extendOutwards == true) {
      this.extenderSubsystem.setSetpoint(.25);
    }
    else {
      this.extenderSubsystem.setSetpoint(0);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.extenderSubsystem.setSetpoint(this.extenderSubsystem.getExtenderPosition());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
