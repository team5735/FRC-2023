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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.TurnMotorFullSpeedCommand;
import frc.robot.commands.BrakeCommand;
import frc.robot.commands.GyroAutocorrectCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.trajectories.Trajectories;

//Intake imports
import frc.robot.commands.intake.*;
import frc.robot.subsystems.IntakeSubsystem;

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
  private final XboxController driverController, subsystemController;
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem swerveSubsystem;
  private final IntakeSubsystem intakeSubsystem;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    this.driverController = new XboxController(Constants.OIConstants.DRIVER_CONTROLLER_PORT);
    this.subsystemController = new XboxController(Constants.OIConstants.SUBSYSTEM_CONTROLLER_PORT);

    this.swerveSubsystem = new SwerveSubsystem();
    this.intakeSubsystem = new IntakeSubsystem();

    // Configure the button bindings
    this.configureDriverBindings();
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
    //     .whenPressed(new InstantCommand(() -> {
    //       this.swerveSubsystem.toggleFieldOriented();
    //     }));

    // NOTE: Strikethrough caused by depreciated, but functional, software
    // FOR TESTING When Y is pressed, trigger gyro autocorrect command
    /*new JoystickButton(this.driverController, XboxController.Button.kY.value)
        .whenHeld(new GyroAutocorrectCommand(this.swerveSubsystem));
    */ 
    new Trigger(this.driverController::getYButton)
      .whileTrue(new GyroAutocorrectCommand(this.swerveSubsystem));

    // When X is pressed, reset gyro to 0
    /*new JoystickButton(this.driverController, XboxController.Button.kX.value)
        .whenPressed(new InstantCommand(() -> {
          this.swerveSubsystem.zeroHeading();
        }));
    */
    new Trigger(this.driverController::getXButton)
        .whileTrue(new InstantCommand(() -> {
          this.swerveSubsystem.zeroHeading();
        }));

    // When B is pressed, make the wheels brake.
    /*new JoystickButton(this.driverController, XboxController.Button.kB.value)
        .whenHeld(new BrakeCommand(this.swerveSubsystem));
    */
    new Trigger(this.driverController::getBButton)
        .whileTrue(new BrakeCommand(this.swerveSubsystem));

    // new JoystickButton(this.driverController, XboxController.Button.kA.value)
    //   .whenHeld(new TurnMotorFullSpeedCommand(swerveSubsystem));

    //INTAKE CONTROLS
    //Right bumper for intake forward
    new Trigger(this.driverController::getRightBumper)
        .whileTrue(new IntakeForward(this.intakeSubsystem))
        .whileFalse(new IntakeStop(this.intakeSubsystem));

    //Left bumper for intake backward
    new Trigger(this.driverController::getLeftBumper)
        .whileTrue(new IntakeBackward(this.intakeSubsystem))
        .whileFalse(new IntakeStop(this.intakeSubsystem));

    //Button A to reverse intake (if that problem happens again...)
  }



  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // The trajectory to follow
    Trajectory trajectory = Trajectories.ONE_METER_STRAIGHT;
    // Trajectory pathweavedTrajectory = Trajectories.generateTrajectory("FieldTest#2.wpilib.json");

    PIDController xController = new PIDController(Constants.AutoConstants.AUTO_XCONTROLLER_KP, 0, 0);
    PIDController yController = new PIDController(Constants.AutoConstants.AUTO_YCONTROLLER_KP, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
        Constants.AutoConstants.AUTO_THETACONTROLLER_KP, 0, 0,
        Constants.AutoConstants.AUTO_THETACONTROLLER_CONSTRAINTS);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        trajectory, // The trajectory to follow
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

  //     RamseteCommand ramseteCommand = new RamseteCommand(
  //       exampleTrajectory,
  //       m_drivetrain::getPose,
  //       new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
  //       new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter, DriveConstants.kaVoltSecondsSquaredPerMeter),
  //       DriveConstants.kDriveKinematics,
  //       m_drivetrain::getWheelSpeeds,
  //       new PIDController(DriveConstants.kPDriveVelLeft, 0, 0),
  //       new PIDController(DriveConstants.kPDriveVelRight, 0, 0),
  //       m_drivetrain::tankDriveVolts,
  //       m_drivetrain);

//     // Set up a sequence of commands
//     // First, we want to reset the drivetrain odometry
//     return new InstantCommand(() -> m_drivetrain.resetOdometry(exampleTrajectory.getInitialPose()), m_drivetrain)
//         // next, we run the actual ramsete command
//         .andThen(ramseteCommand)

//         // Finally, we make sure that the robot stops
//         .andThen(new InstantCommand(() -> m_drivetrain.tankDriveVolts(0, 0), m_drivetrain));
//   } 
// }
