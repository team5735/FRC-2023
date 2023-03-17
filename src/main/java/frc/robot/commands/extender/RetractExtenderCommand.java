// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.extender;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.Constants.ExtenderConstants;

/**
 * Bring the extender back into 0 position
 */
public class RetractExtenderCommand extends CommandBase {

  private final ExtenderSubsystem extenderSubsystem;
  /** Creates a new ExtenderIn. */
  public RetractExtenderCommand(ExtenderSubsystem extenderSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.extenderSubsystem = extenderSubsystem;
    addRequirements(extenderSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.extenderSubsystem.setLevel(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Is extender fully retracted? " + this.extenderSubsystem.isFullyRetracted());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.extenderSubsystem.isFullyRetracted();
  }
}
