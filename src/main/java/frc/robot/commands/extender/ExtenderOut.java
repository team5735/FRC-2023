// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.extender;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.Constants.ExtenderConstants;

public class ExtenderOut extends CommandBase {
  /** Creates a new ExtenderOut. */

  private final ExtenderSubsystem extenderSubsystem;
  private final int desiredLevel;

  private double goalEncoderPosition;

  public ExtenderOut(ExtenderSubsystem extenderSubsystem, int desiredLevel) {
    this.extenderSubsystem = extenderSubsystem;
    this.desiredLevel = desiredLevel;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(extenderSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    extenderSubsystem.extenderMove(ExtenderConstants.EXTEND_MOTOR_OUT_SPEED);

    //Do we want this, or do we want the values to just be permanent (not based on current at all)????
    goalEncoderPosition = extenderSubsystem.getCurrentEncoderPosition() + extenderSubsystem.setGoalEncoderPosition(desiredLevel);

    SmartDashboard.putString("out ext", "running");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("encoder", extenderSubsystem.getCurrentEncoderPosition());
    //Check if current encoder position at goal encoder position
    if (extenderSubsystem.getCurrentEncoderPosition() > (ExtenderConstants.EXTEND_MAX_ENCODER - ExtenderConstants.EXTEND_SLOW_DISTANCE) ||
        //Make sure never surpasses max extension
        (extenderSubsystem.getCurrentEncoderPosition() > (goalEncoderPosition - ExtenderConstants.EXTEND_SLOW_DISTANCE) &&
        extenderSubsystem.getCurrentEncoderPosition() < (goalEncoderPosition + ExtenderConstants.EXTEND_SLOW_DISTANCE))) {
          SmartDashboard.putString("target", "attained");
          extenderSubsystem.extenderStop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
