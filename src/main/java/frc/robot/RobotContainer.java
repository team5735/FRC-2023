// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
//import edu.wpi.first.wpilibj2.command.button.JoystickButton; //Outdated

import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.OIConstants;

// Commands Import
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.BrakeCommand;
import frc.robot.commands.extender.*;
import frc.robot.commands.GyroAutocorrectCommand;
import frc.robot.commands.ManualElevatorCmd;
// Pneumatics Imports -- Could be reorganized by system
import frc.robot.commands.vision.TurnToZero;
import frc.robot.subsystems.PneumaticsSubsystem;

import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
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
  private final CommandXboxController driverController, subsystemController;
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem swerveSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final PneumaticsSubsystem pneumaticsSubsystem;
  private final ExtenderSubsystem extenderSubsystem;
  private final VisionSubsystem visionSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;

  private Map<String, Command> eventMap = new HashMap<>();

  private final SendableChooser<String> autoPathChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands(?)
   */
  public RobotContainer() {
    this.swerveSubsystem = new SwerveSubsystem();
    this.intakeSubsystem = new IntakeSubsystem();
    this.pneumaticsSubsystem = new PneumaticsSubsystem();
    this.extenderSubsystem = new ExtenderSubsystem();
    this.elevatorSubsystem = new ElevatorSubsystem();

    // pipeline stuff not set up yet (do we even need?)
    this.visionSubsystem = new VisionSubsystem(0);

    this.driverController = new CommandXboxController(Constants.OIConstants.DRIVER_CONTROLLER_PORT);
    this.subsystemController = new CommandXboxController(Constants.OIConstants.SUBSYSTEM_CONTROLLER_PORT);

    // Configure the button bindings
    this.configureDriverBindings();
    this.configureSubsystemBindings();

    this.populateEventMap();

    this.populatePathChooser();
  }

  private void populatePathChooser() {
    // Read files in deploy/pathplanner directory
    try {
      try (Stream<Path> walk = Files.walk(Filesystem.getDeployDirectory().toPath().resolve("pathplanner"))) {
        List<String> fileNames = walk
            .filter(p -> !Files.isDirectory(p))
            .map(p -> p.getFileName().toString()) // turns files into file names
            .filter(f -> f.endsWith(".path")) // gets ones ending in .path
            .map(n -> n.split(".path")[0]) // gets the file name excluding .path
            .collect(Collectors.toList());

        for (String pathName : fileNames) {
          this.autoPathChooser.addOption(pathName, pathName);
        }
    
        this.autoPathChooser.setDefaultOption(
            fileNames.get(0),
            fileNames.get(0));
      }
    } catch (Exception e) {
      System.out.println("##### ERROR: No paths found!");
    }
    
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
        // TODO: check if this direction is correct
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

    // Button A to reverse intake (if that problem happens again...)

    // turn to zero
    // this.driverController.back()
    // .whileTrue(new TurnToZero(visionSubsystem, swerveSubsystem));
  }

  private void configureSubsystemBindings() {
    // Button Start (diagonal bottom right from Xbox center button) on Subsystem
    // Controller to trigger Compressor On (implement on/off)
    this.subsystemController.start()
        .whileTrue(new InstantCommand(() -> {
          this.pneumaticsSubsystem.toggleCompressor();
        }));

    this.subsystemController.a()
        .whileTrue(new InstantCommand(() -> {
          this.pneumaticsSubsystem.togglePiston();
        }));

    // COMMAND: Brings to level 0 (bottom)
    subsystemController.x()
    .toggleOnTrue(new SequentialCommandGroup(
    new RetractExtenderCommand(this.extenderSubsystem),
    new InstantCommand(() -> {
    this.elevatorSubsystem.setLevel(0);
    })));

    // COMMAND: Brings to level 1 (bottom scoring level)
    // subsystemController.y()
    //   .toggleOnTrue(new ParallelCommandGroup(
    //   new InstantCommand(() -> {
    //   this.elevatorSubsystem.setLevel(1);
    // }),
    // new SequentialCommandGroup(
    //     new WaitCommand(0.5),
    //     new InstantCommand(() -> {
    //     this.extenderSubsystem.setLevel(1);
    // }))));

    // COMMAND: Brings to level 2 (middle scoring level);
    this.subsystemController.y()
    .toggleOnTrue(new ParallelCommandGroup(
    new InstantCommand(() -> {
    this.elevatorSubsystem.setLevel(2);
    }),
    new InstantCommand(() -> {
    this.extenderSubsystem.setLevel(2);
    })));

    this.subsystemController.b()
    .toggleOnTrue(
    new InstantCommand(() -> {
      this.elevatorSubsystem.setLevel(2);
    }));

    // this.subsystemController.rightTrigger()
    // .whileTrue(new TurnToZero(visionSubsystem, swerveSubsystem));

    this.extenderSubsystem.setDefaultCommand(
        new ManualExtenderCmd(this.extenderSubsystem,
            () -> this.subsystemController.getRightTriggerAxis() - this.subsystemController.getLeftTriggerAxis(),
            () -> this.elevatorSubsystem.getElevatorHeight()));

    // Left + Right bumpers: Decrease / Increase elevator setpoint
    this.subsystemController.rightBumper().toggleOnTrue(new InstantCommand(() -> {
      this.elevatorSubsystem.setSetpoint(this.elevatorSubsystem.getSetpoint() + 0.02);
    }));

    this.subsystemController.leftBumper().toggleOnTrue(new InstantCommand(() -> {
      this.elevatorSubsystem.setSetpoint(this.elevatorSubsystem.getSetpoint() - 0.02);
    }));

    // this.elevatorSubsystem.setDefaultCommand(
    // new ManualElevatorCmd(this.elevatorSubsystem,
    // () -> -this.subsystemController.getRightY())); // negative b/c y is inverted
  }

  private void populateEventMap() {
    // Carson: Put the event marker name and the command to run, like this:
    // TODO: Then put the event markers with these exact names in the path
    eventMap.put("toggleCompressor", new InstantCommand(() -> {
      this.pneumaticsSubsystem.toggleCompressor();
    }));

    eventMap.put("togglePiston", new InstantCommand(() -> {
      this.pneumaticsSubsystem.togglePiston();
    }));

    eventMap.put("level0", new SequentialCommandGroup(
        new RetractExtenderCommand(this.extenderSubsystem),
        new InstantCommand(() -> {
          this.elevatorSubsystem.setLevel(0);
        })));

    eventMap.put("level1", new ParallelCommandGroup(
        new InstantCommand(() -> {
          this.elevatorSubsystem.setLevel(1);
        }),
        new SequentialCommandGroup(
            new WaitCommand(1),
            new InstantCommand(() -> {
              this.extenderSubsystem.setLevel(1);
            }))));

    eventMap.put("level2", new ParallelCommandGroup(
        new InstantCommand(() -> {
          this.elevatorSubsystem.setLevel(2);
        }),
        new SequentialCommandGroup(
            new WaitCommand(1),
            new InstantCommand(() -> {
              this.extenderSubsystem.setLevel(2);
            }))));

    eventMap.put("runIntakeIn",
        new ParallelDeadlineGroup(
            new WaitCommand(1),
            // Run the intake
            new IntakeCommand(this.intakeSubsystem, IntakeDirection.IN)));

    eventMap.put("runIntakeOut",
        new ParallelDeadlineGroup(
            new WaitCommand(1),
            // Run the intake
            new IntakeCommand(this.intakeSubsystem, IntakeDirection.OUT)));

    eventMap.put("gyroBalance",
        new GyroAutocorrectCommand(swerveSubsystem));
        
    eventMap.put("brake",
        new BrakeCommand(swerveSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // The trajectory to follow
    PathPlannerTrajectory plotTrajectory
    // = Trajectories.ONE_METER_STRAIGHT;
    // = Trajectories.ONE_METER_STRAIGHT;
        = Trajectories.loadTrajectory(this.autoPathChooser.getSelected());

    // PIDController xController = new
    // PIDController(Constants.AutoConstants.AUTO_XCONTROLLER_KP, 0, 0);
    // PIDController yController = new
    // PIDController(Constants.AutoConstants.AUTO_YCONTROLLER_KP, 0, 0);
    // PIDController thetaController = new
    // PIDController(Constants.AutoConstants.AUTO_THETACONTROLLER_KP, 0, 0);

    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
        swerveSubsystem::getPose,
        swerveSubsystem::resetOdometry,
        Constants.DrivetrainConstants.DT_KINEMATICS,
        new PIDConstants(Constants.AutoConstants.AUTO_XCONTROLLER_KP, 0, 0),
        new PIDConstants(Constants.AutoConstants.AUTO_THETACONTROLLER_KP, 0, 0),
        new Consumer<SwerveModuleState[]>() {
          @Override
          public void accept(SwerveModuleState[] states) {
            swerveSubsystem.setModuleStates(states, false);
          }
        },
        eventMap,
        true,
        swerveSubsystem);

    return autoBuilder.fullAuto(plotTrajectory);

    // PPSwerveControllerCommand swerveControllerCommand = new
    // PPSwerveControllerCommand(
    // plotTrajectory,
    // swerveSubsystem::getPose, // Pose supplier
    // Constants.DrivetrainConstants.DT_KINEMATICS, // SwerveDriveKinematics
    // xController,
    // yController,
    // thetaController,
    // new Consumer<SwerveModuleState[]>() {
    // @Override
    // public void accept(SwerveModuleState[] states) {
    // swerveSubsystem.setModuleStates(states, false);
    // }
    // }, // Module states consumer
    // true, // Should the path be automatically mirrored depending on alliance
    // color.
    // // Optional, defaults to true
    // swerveSubsystem // Requires this drive subsystem
    // );

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

    // return new SequentialCommandGroup(
    // // Start the command by "placing" the robot at the beginning of the
    // trajectory
    // new InstantCommand(() -> swerveSubsystem.resetOdometry(
    // plotTrajectory.getInitialHolonomicPose())),
    // // Run the trajectory command
    // new FollowPathWithEvents(
    // swerveControllerCommand,
    // plotTrajectory.getMarkers(),
    // eventMap)
    // // ,
    // // // Stop the robot at the end of the trajectory
    // new InstantCommand(() -> swerveSubsystem.stopModules())
    // );
  }

  /**
   * Resets setpoints to 0 on disable and init
   */
  public void resetSetpoints() {
    this.elevatorSubsystem.setSetpoint(0);
    this.extenderSubsystem.setSetpoint(0);
  }
}
