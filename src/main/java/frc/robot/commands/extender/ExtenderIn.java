// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.extender;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.Constants.ExtenderConstants;

public class ExtenderIn extends CommandBase {

  private final ExtenderSubsystem extenderSubsystem;
  /** Creates a new ExtenderIn. */
  public ExtenderIn(ExtenderSubsystem extenderSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.extenderSubsystem = extenderSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    extenderSubsystem.extenderMove(ExtenderConstants.EXTEND_MOTOR_IN_SPEED);
    SmartDashboard.putString("in ext", "running");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (extenderSubsystem.getCurrentEncoderPosition() < (ExtenderConstants.EXTEND_MIN_ENCODER + ExtenderConstants.EXTEND_SLOW_DISTANCE)) {
          extenderSubsystem.extenderStop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    extenderSubsystem.extenderStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
