// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Consumer;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// Commands Import
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.BrakeCommand;
import frc.robot.commands.ExtendCommand;
import frc.robot.commands.GyroAutocorrectCommand;
import frc.robot.commands.ManualElevatorCmd;
// Pneumatics Imports -- Could be reorganized by system
import frc.robot.subsystems.PneumaticsSubsystem;

import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.trajectories.Trajectories;

// Intake imports
import frc.robot.commands.intake.*;
import frc.robot.commands.intake.IntakeCommand.IntakeDirection;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExtenderSubsystem;

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
  private final CommandXboxController driverController;
  private final CommandXboxController subsystemController;
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem swerveSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final PneumaticsSubsystem pneumaticsSubsystem;
  private final ExtenderSubsystem extenderSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands(?)
   */
  public RobotContainer() {
    this.swerveSubsystem = new SwerveSubsystem();
    this.intakeSubsystem = new IntakeSubsystem();
    this.pneumaticsSubsystem = new PneumaticsSubsystem();
    this.extenderSubsystem = new ExtenderSubsystem();
    this.elevatorSubsystem = new ElevatorSubsystem();

    this.driverController = new CommandXboxController(Constants.OIConstants.DRIVER_CONTROLLER_PORT);
    this.subsystemController = new CommandXboxController(Constants.OIConstants.SUBSYSTEM_CONTROLLER_PORT);

    // Configure the button bindings
    this.configureDriverBindings();
    this.configureSubsystemBindings();
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
        () -> -this.driverController.getRightX())); // Rotate left on controller is - but rotate left on controller is +

    // When Y is pressed on driver controller, toggle field oriented
    this.driverController.y()
        .whileTrue(new InstantCommand(() -> {
          this.swerveSubsystem.toggleFieldOriented();
        }));

    // NOTE: Strikethrough caused by deprecated, but functional, software
    // FOR TESTING When Y is pressed, trigger gyro autocorrect command
    /*
     * new JoystickButton(this.driverController, XboxController.Button.kY.value)
     * .whenHeld(new GyroAutocorrectCommand(this.swerveSubsystem));
     */
    // this.driverController.y()
    // .whileTrue(new GyroAutocorrectCommand(this.swerveSubsystem));

    // When X is pressed, reset gyro to 0
    this.driverController.x()
        .whileTrue(new InstantCommand(() -> {
          this.swerveSubsystem.zeroHeading();
        }));

    // When B is pressed, make the wheels brake.
    this.driverController.b()
        .whileTrue(new BrakeCommand(this.swerveSubsystem));

    // INTAKE CONTROLS
    // Right bumper for intake forward
    this.driverController.rightBumper()
        .whileTrue(new IntakeCommand(this.intakeSubsystem, IntakeDirection.FORWARD));

    // Left bumper for intake backward
    this.driverController.leftBumper()
        .whileTrue(new IntakeCommand(this.intakeSubsystem, IntakeDirection.BACKWARD));

    // Button A to reverse intake (if that problem happens again...)
  }

  private void configureSubsystemBindings() {
    // Button A on Subsystem Controller to trigger Compressor On (implement on/off)
    // this.subsystemController.a()
    // .whileTrue(new InstantCommand(() -> {
    // this.pneumaticsSubsystem.toggleCompressor();
    // }));

    // this.subsystemController.leftBumper()
    // .whileTrue(new InstantCommand(() -> {
    // this.pneumaticsSubsystem.togglePiston();
    // }));

    // TODO: Use parallel command group to run elevator and extender at the same
    // time
    // TODO: Determine if this Trigger is reasonable (shortcutted for convenience)
    // () -> Creates a function, lambda operator
    // :: similar to a lambda

    // this.subsystemController.leftStick()
    // .whileTrue(new ExtendCommand(extenderSubsystem, () ->
    // this.subsystemController.getLeftY()));

    this.elevatorSubsystem.setDefaultCommand(
        new ManualElevatorCmd(this.elevatorSubsystem,
            () -> -this.subsystemController.getRightY())); // negative b/c y is inverted
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // The trajectory to follow
    PathPlannerTrajectory plotTrajectory = Trajectories.ONE_METER_STRAIGHT;
    // = Trajectories.ONE_METER_STRAIGHT;
    // = Trajectories.loadTrajectory("ForwardMove.wpilib.json");

    PIDController xController = new PIDController(Constants.AutoConstants.AUTO_XCONTROLLER_KP, 0, 0);
    PIDController yController = new PIDController(Constants.AutoConstants.AUTO_YCONTROLLER_KP, 0, 0);
    PIDController thetaController = new PIDController(Constants.AutoConstants.AUTO_THETACONTROLLER_KP, 0, 0);
    // ProfiledPIDController thetaController = new ProfiledPIDController(
    // Constants.AutoConstants.AUTO_THETACONTROLLER_KP, 0, 0,
    // Constants.AutoConstants.AUTO_THETACONTROLLER_CONSTRAINTS);
    // thetaController.enableContinuousInput(-Math.PI, Math.PI);

    PPSwerveControllerCommand swerveControllerCommand = new PPSwerveControllerCommand(
        plotTrajectory,
        swerveSubsystem::getPose, // Pose supplier
        Constants.DrivetrainConstants.DT_KINEMATICS, // SwerveDriveKinematics
        xController,
        yController,
        thetaController,
        new Consumer<SwerveModuleState[]>() {
          @Override
          public void accept(SwerveModuleState[] states) {
            swerveSubsystem.setModuleStates(states, false);
          }
        }, // Module states consumer
        true, // Should the path be automatically mirrored depending on alliance color.
              // Optional, defaults to true
        swerveSubsystem // Requires this drive subsystem
    );

    // SwerveControllerCommand swerveControllerCommand = new
    // SwerveControllerCommand(
    // plotTrajectory, // The trajectory to follow
    // swerveSubsystem::getPose, // The supplier of the robot's x/y position and
    // heading
    // Constants.DrivetrainConstants.DT_KINEMATICS, // The kinematics of the robot
    // xController, // The PID controller to correct error in the robot's x position
    // yController, // The PID controller to correct error in the robot's y position
    // thetaController, // The PID controller to correct error in the robot's
    // heading
    // swerveSubsystem::setModuleStates, // The function to use to set the robot's
    // module states
    // swerveSubsystem); // The subsystem to execute the command on

    return new SequentialCommandGroup(
        // Start the command by "placing" the robot at the beginning of the trajectory
        new InstantCommand(() -> swerveSubsystem.resetOdometry(
            plotTrajectory.getInitialHolonomicPose())),
        // Run the trajectory command
        swerveControllerCommand,
        // Stop the robot at the end of the trajectory
        new InstantCommand(() -> swerveSubsystem.stopModules()));
  }
}
