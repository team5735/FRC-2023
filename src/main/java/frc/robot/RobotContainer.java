// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
//import edu.wpi.first.wpilibj2.command.button.JoystickButton; //Outdated

import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.OIConstants;

// Commands Import
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.TurnMotorFullSpeedCommand;
import frc.robot.commands.BrakeCommand;
import frc.robot.commands.extender.*;
import frc.robot.commands.GyroAutocorrectCommand;

// Pneumatics Imports -- Could be reorganized by system
import frc.robot.commands.pneumatics.CompressorOnOff;
import frc.robot.commands.pneumatics.extendRetract;
import frc.robot.commands.vision.TurnToZero;
import frc.robot.subsystems.PneumaticsSubsystem;

import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.trajectories.Trajectories;

// Intake imports
import frc.robot.commands.intake.*;
import frc.robot.subsystems.IntakeSubsystem;

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
  private final CommandXboxController driverController, subsystemController;
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem swerveSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final PneumaticsSubsystem pneumaticsSubsystem;
  private final ExtenderSubsystem extenderSubsystem;
  private final VisionSubsystem visionSubsystem;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands(?)
   */
  public RobotContainer() {
    this.swerveSubsystem = new SwerveSubsystem();
    this.intakeSubsystem = new IntakeSubsystem();
    this.pneumaticsSubsystem = new PneumaticsSubsystem();
    this.extenderSubsystem = new ExtenderSubsystem();

    // pipeline stuff not set up yet (do we even need?)
    this.visionSubsystem = new VisionSubsystem(0);

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
    // new JoystickButton(this.driverController, XboxController.Button.kY.value)
    // .whenPressed(new InstantCommand(() -> {
    // this.swerveSubsystem.toggleFieldOriented();
    // }));

    // NOTE: Strikethrough caused by depreciated, but functional, software
    // FOR TESTING When Y is pressed, trigger gyro autocorrect command
    /*
     * new JoystickButton(this.driverController, XboxController.Button.kY.value)
     * .whenHeld(new GyroAutocorrectCommand(this.swerveSubsystem));
     */
    driverController.y()
        .whileTrue(new GyroAutocorrectCommand(this.swerveSubsystem));

    // When X is pressed, reset gyro to 0
    /*
     * new JoystickButton(this.driverController, XboxController.Button.kX.value)
     * .whenPressed(new InstantCommand(() -> {
     * this.swerveSubsystem.zeroHeading();
     * }));
     */
    driverController.x()
        .whileTrue(new InstantCommand(() -> {
          this.swerveSubsystem.zeroHeading();
        }));

    // When B is pressed, make the wheels brake.
    /*
     * new JoystickButton(this.driverController, XboxController.Button.kB.value)
     * .whenHeld(new BrakeCommand(this.swerveSubsystem));
     */
    driverController.b()
        .whileTrue(new BrakeCommand(this.swerveSubsystem));

    // new JoystickButton(this.driverController, XboxController.Button.kA.value)
    // .whenHeld(new TurnMotorFullSpeedCommand(swerveSubsystem));

    // INTAKE CONTROLS
    // Right bumper for intake forward
    driverController.rightBumper()
        .whileTrue(new IntakeForward(this.intakeSubsystem))
        .whileFalse(new IntakeStop(this.intakeSubsystem));

    // Left bumper for intake backward
    driverController.leftBumper()
        .whileTrue(new IntakeBackward(this.intakeSubsystem))
        .whileFalse(new IntakeStop(this.intakeSubsystem));

    // Button A to reverse intake (if that problem happens again...)

    // turn to zero
    driverController.back()
        .whileTrue(new TurnToZero(visionSubsystem, swerveSubsystem));
  }

  private void configureSubsystemBindings() {
    // Button A on Subsystem Controller to trigger Compressor On (implement on/off)
    subsystemController.a()
        .whileTrue(new CompressorOnOff(this.pneumaticsSubsystem));

    subsystemController.leftBumper()
        .whileTrue(new extendRetract(this.pneumaticsSubsystem));

    // TODO: Use parallel command group to run elevator and extender at the same
    // time
    // TODO: Determine if this Trigger is reasonable (shortcutted for convenience)
    // () -> Creates a function, lambda operator
    // :: similar to a lambda
    // new Trigger(() -> this.subsystemController.getLeftY() > 0.08 ||
    // this.subsystemController.getLeftY() < -0.08)
    // .whileTrue(new ExtenderControl(extenderSubsystem, () ->
    // this.subsystemController.getLeftY()));

    // EXTENDER CONTROLS
    // //X button to extend to low level
    // new Trigger(this.subsystemController::getXButtonPressed)
    // .onTrue(new ExtenderOut(this.extenderSubsystem, 1));

    subsystemController.x()
        .whileTrue(new ExtenderOut(this.extenderSubsystem, 2))
        .whileFalse(new ExtenderStop(this.extenderSubsystem));

    // //Y button to extend to mid level
    // new Trigger(this.subsystemController::getYButtonPressed)
    // .onTrue(new ExtenderOut(this.extenderSubsystem, 2));

    // //B button to extend to upper level
    // new Trigger(this.subsystemController::getBButtonPressed)
    // .onTrue(new ExtenderOut(this.extenderSubsystem, 3));

    // //right bumper to bring all the way back to start
    // new Trigger(this.subsystemController::getRightBumperPressed)
    // .onTrue(new ExtenderIn(this.extenderSubsystem));

    // //right stick button to stop extender
    // new Trigger(this.subsystemController::getRightStickButtonPressed)
    // .onTrue(new ExtenderStop(this.extenderSubsystem));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // The trajectory to follow
    Trajectory plotTrajectory = Trajectories.ONE_METER_STRAIGHT;
    Trajectory pathweavedTrajectory = Trajectories.generateTrajectory("ForwardMove.wpilib.json");

    PIDController xController = new PIDController(Constants.AutoConstants.AUTO_XCONTROLLER_KP, 0, 0);
    PIDController yController = new PIDController(Constants.AutoConstants.AUTO_YCONTROLLER_KP, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
        Constants.AutoConstants.AUTO_THETACONTROLLER_KP, 0, 0,
        Constants.AutoConstants.AUTO_THETACONTROLLER_CONSTRAINTS);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        plotTrajectory, // The trajectory to follow
        swerveSubsystem::getPose, // The supplier of the robot's x/y position and heading
        Constants.DrivetrainConstants.DT_KINEMATICS, // The kinematics of the robot
        xController, // The PID controller to correct error in the robot's x position
        yController, // The PID controller to correct error in the robot's y position
        thetaController, // The PID controller to correct error in the robot's heading
        swerveSubsystem::setModuleStates, // The function to use to set the robot's module states
        swerveSubsystem); // The subsystem to execute the command on

    return new SequentialCommandGroup(
        // Start the command by "placing" the robot at the beginning of the trajectory
        new InstantCommand(() -> swerveSubsystem.resetOdometry(
        // trajectory.getInitialPose()
        )),
        // Run the trajectory command
        swerveControllerCommand,
        // Stop the robot at the end of the trajectory
        new InstantCommand(() -> swerveSubsystem.stopModules()));
  }
}

// private Command generateRamseteCommand() {

// RamseteCommand ramseteCommand = new RamseteCommand(
// exampleTrajectory,
// m_drivetrain::getPose,
// new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
// new SimpleMotorFeedforward(DriveConstants.ksVolts,
// DriveConstants.kvVoltSecondsPerMeter,
// DriveConstants.kaVoltSecondsSquaredPerMeter),
// DriveConstants.kDriveKinematics,
// m_drivetrain::getWheelSpeeds,
// new PIDController(DriveConstants.kPDriveVelLeft, 0, 0),
// new PIDController(DriveConstants.kPDriveVelRight, 0, 0),
// m_drivetrain::tankDriveVolts,
// m_drivetrain);

// // Set up a sequence of commands
// // First, we want to reset the drivetrain odometry
// return new InstantCommand(() ->
// m_drivetrain.resetOdometry(exampleTrajectory.getInitialPose()), m_drivetrain)
// // next, we run the actual ramsete command
// .andThen(ramseteCommand)

// // Finally, we make sure that the robot stops
// .andThen(new InstantCommand(() -> m_drivetrain.tankDriveVolts(0, 0),
// m_drivetrain));
// }
// }
