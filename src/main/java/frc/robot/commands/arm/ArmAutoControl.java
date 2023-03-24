package frc.robot.commands.arm;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

/**
 * Bring the extender back into 0 position
 */
public class ArmAutoControl extends CommandBase {

  private final ArmSubsystem armSubsystem;
  private int level;
  /** Creates a new ExtenderIn. */
  public ArmAutoControl(ArmSubsystem armSubsystem, int level) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.armSubsystem = armSubsystem;
    this.level = level;
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.armSubsystem.setLevel(level);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // System.out.println("Is extender fully retracted? " + this.armSubsystem.isFullyRetracted());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.armSubsystem.isAtSetpoint();
  }
}
