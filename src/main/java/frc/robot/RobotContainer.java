// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.auto.AutoCommands;
import frc.robot.commands.grabber.GrabberCommand;
import frc.robot.commands.grabber.GrabberCommand.GrabberDirection;
// Pneumatics Imports -- Could be reorganized by system
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.trajectories.Trajectories;

// Intake imports
import frc.robot.commands.intake.*;
import frc.robot.commands.intake.IntakeCommand.IntakeDirection;
import frc.robot.commands.swerve.BrakeCommand;
import frc.robot.commands.swerve.GyroAutocorrectCommand;
import frc.robot.commands.swerve.SwerveJoystickCmd;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final CommandXboxController driverController;//, subsystemController;
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem swerveSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final ArmSubsystem armSubsystem;
  private final GrabberSubsystem grabberSubsystem;

  private final SendableChooser<String> autoPathChooser = new SendableChooser<>();

  private final AutoCommands autoCommands;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands(?)
   */
  public RobotContainer() {
    this.swerveSubsystem = new SwerveSubsystem();
    this.intakeSubsystem = new IntakeSubsystem();
    this.armSubsystem = new ArmSubsystem();
    this.grabberSubsystem = new GrabberSubsystem();

    this.driverController = new CommandXboxController(Constants.OIConstants.DRIVER_CONTROLLER_PORT);
    // this.subsystemController = new CommandXboxController(Constants.OIConstants.SUBSYSTEM_CONTROLLER_PORT);

    this.autoCommands = new AutoCommands(swerveSubsystem, intakeSubsystem, armSubsystem, grabberSubsystem);

    // Configure the button bindings
    this.configureDriverBindings();
    this.configureSubsystemBindings();

    this.populatePathChooser();
  }

  // Populates it with the Auto Cmd Map
  private void populatePathChooser() {
    for (String cmdName : this.autoCommands.AUTO_CMD_MAP.keySet()) {
      this.autoPathChooser.addOption(cmdName, cmdName);
    }

    this.autoPathChooser.setDefaultOption(
        this.autoCommands.AUTO_CMD_MAP.keySet().stream().findFirst().get(),
        this.autoCommands.AUTO_CMD_MAP.keySet().stream().findFirst().get());

    SmartDashboard.putData("Auto Path Chooser", this.autoPathChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureDriverBindings() {
    // When there's no other command scheduled, drive the robot with Xbox joysticks
    this.swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(this.swerveSubsystem,
        () -> -this.driverController.getLeftY(), // Forward on controller is -Y but forward on robot is X.
        () -> -this.driverController.getLeftX(), // Left on controller is -X but left on robot is +Y.
        () -> (-driverController.getRightTriggerAxis() + driverController.getLeftTriggerAxis()),
        () -> this.driverController.a().getAsBoolean())); // turbohack for
                                                          // ergonomics

    // When Y is pressed on driver controller, toggle field oriented
    this.driverController.y()
        .whileTrue(new InstantCommand(() -> {
          this.swerveSubsystem.toggleFieldOriented();
        }));

    // When X is pressed, reset gyro to 0
    this.driverController.x()
        .whileTrue(new InstantCommand(() -> {
          this.swerveSubsystem.zeroHeading();
        }));

    // When B is pressed, make the wheels brake.
    this.driverController.b()
        .whileTrue(new BrakeCommand(this.swerveSubsystem));

    // INTAKE CONTROLS
    this.driverController.rightBumper()
        .whileTrue(new IntakeCommand(this.intakeSubsystem, IntakeDirection.IN));

    // // Left bumper for intake backward
    this.driverController.leftBumper()
        .whileTrue(new IntakeCommand(this.intakeSubsystem, IntakeDirection.OUT));

    this.driverController.start()
        .whileTrue(new GyroAutocorrectCommand(this.swerveSubsystem));
  }

  private void configureSubsystemBindings() {
    // this.subsystemController.a()
    //     .whileTrue(new GrabberCommand(grabberSubsystem, GrabberDirection.IN));

    // this.subsystemController.b()
    //     .whileTrue(new GrabberCommand(grabberSubsystem, GrabberDirection.OUT));

    // this.subsystemController.rightTrigger()
    //     .whileTrue(new GrabberCommand(grabberSubsystem, GrabberDirection.SLOW_OUT));

    // this.subsystemController.x()
    //     .toggleOnTrue(new InstantCommand(() -> {
    //       this.armSubsystem.setLevel(0);
    //     }));

    // this.subsystemController.y()
    //     .toggleOnTrue(new InstantCommand(() -> {
    //       this.armSubsystem.setLevel(2);
    //     }));

    // // Left + Right bumpers: Decrease / Increase elevator setpoint
    // this.subsystemController.rightBumper().toggleOnTrue(new InstantCommand(() -> {
    //   this.armSubsystem.setSetpoint(this.armSubsystem.getSetpoint() + 0.1);
    // }));

    // this.subsystemController.leftBumper().toggleOnTrue(new InstantCommand(() -> {
    //   this.armSubsystem.setSetpoint(this.armSubsystem.getSetpoint() - 0.1);
    // }));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    String selected = this.autoPathChooser.getSelected();
    return this.autoCommands.AUTO_CMD_MAP.get(selected).get();
  }

  /**
   * Resets setpoints to 0 on disable and init
   */
  public void resetSetpoints() {
    this.armSubsystem.resetMotors();
    this.armSubsystem.setSetpoint(Constants.ArmConstants.ARM_POSITION_START_RAD);
  }
}
