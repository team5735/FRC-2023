package frc.robot.commands.auto;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.arm.ArmAutoControl;
import frc.robot.commands.grabber.GrabberCommand;
import frc.robot.commands.grabber.GrabberCommand.GrabberDirection;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.intake.IntakeCommand.IntakeDirection;
import frc.robot.commands.swerve.BrakeCommand;
import frc.robot.commands.swerve.GyroAutocorrectCommand;
import frc.robot.commands.swerve.MoveStraightCmd;
import frc.robot.commands.swerve.MoveStraightCmd.MoveDirection;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.trajectories.Trajectories;

public class AutoCommands {

        private final SwerveSubsystem swerveSubsystem;
        private final IntakeSubsystem intakeSubsystem;
        private final ArmSubsystem armSubsystem;
        private final GrabberSubsystem grabberSubsystem;

        public Map<String, Command> eventMap = new HashMap<>();

        public AutoCommands(SwerveSubsystem swerveSubsystem, IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem,
                        GrabberSubsystem grabberSubsystem) {
                this.swerveSubsystem = swerveSubsystem;
                this.intakeSubsystem = intakeSubsystem;
                this.armSubsystem = armSubsystem;
                this.grabberSubsystem = grabberSubsystem;

                this.eventMap = new HashMap<>();
                eventMap.put("runIntakeIn",
                                new ParallelDeadlineGroup(
                                                new WaitCommand(3),
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

        // ===== COMMANDS ===== //
        public Command PlaceMidCone() {
                return new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                                new ParallelDeadlineGroup( // Move backward for 0.5 seconds to allow arm
                                                                new WaitCommand(0.85),
                                                                new MoveStraightCmd(this.swerveSubsystem,
                                                                                MoveDirection.BACKWARD)),
                                                new SequentialCommandGroup(
                                                                new WaitCommand(0.5),
                                                                // Arm raise to mid cone
                                                                new ArmAutoControl(armSubsystem, 1))),
                                new ParallelDeadlineGroup( // Move forward for 0.5 seconds
                                                new WaitCommand(0.2),
                                                new MoveStraightCmd(swerveSubsystem, MoveDirection.FORWARD)),
                                new ParallelDeadlineGroup( // Runs grabber for 0.25 seconds, place cone
                                                new WaitCommand(0.5),
                                                new GrabberCommand(grabberSubsystem, GrabberDirection.SLOW_OUT)));
        }

        public Command PlaceConeGrabCubeAndSpit() {
                return new SequentialCommandGroup(
                                this.PlaceMidCone(), // Place the cone
                                new ParallelDeadlineGroup(
                                                this.getTrajectoryCommand("PlaceConeMoveOutGrabNewBlockMoveBackPart1"),
                                                new ArmAutoControl(armSubsystem, 0), // Arm bring back down
                                                new IntakeCommand(intakeSubsystem, IntakeDirection.IN)),
                                // new ParallelDeadlineGroup( SEE IF PATH CAN DO IT FINE
                                //                 new WaitCommand(1.7),
                                //                 new MoveStraightCmd(swerveSubsystem, MoveDirection.BACKWARD),
                                //                 new IntakeCommand(intakeSubsystem, IntakeDirection.IN)),
                                this.getTrajectoryCommand("PlaceConeMoveOutGrabNewBlockMoveBackPart2"),
                                // Runs intake for 0.5 seconds
                                new ParallelDeadlineGroup(
                                                new WaitCommand(1),
                                                new IntakeCommand(intakeSubsystem, IntakeDirection.OUT)));
        };

        public Command PlaceConeAndBalance() {
                return new SequentialCommandGroup(
                                this.PlaceMidCone(), // Place the cone
                                new ParallelDeadlineGroup(
                                                // Backwards because placing cone first
                                                this.getTrajectoryCommand("BackwardsOverStationAndBacktoBalance"),
                                                // Arm bring back down
                                                new ArmAutoControl(armSubsystem, 0)),
                                new GyroAutocorrectCommand(swerveSubsystem));
        }

        // ===== HELPER ===== //
        public Command getTrajectoryCommand(String fileName) {
                // The trajectory to follow
                // TODO: Add Constraints as part of the path
                PathPlannerTrajectory plotTrajectory = Trajectories.loadTrajectory(fileName);

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
                                this.eventMap,
                                true,
                                swerveSubsystem);

                return autoBuilder.fullAuto(plotTrajectory);
        }

        // The dropdown select options
        public Map<String, Supplier<Command>> AUTO_CMD_MAP = Map
                        .of(
                                        "PlaceConeGrabCubeAndSpit", () -> {
                                                return this.PlaceConeGrabCubeAndSpit();
                                        },
                                        "PlaceConeAndBalance", () -> {
                                                return this.PlaceConeAndBalance();
                                        }//,
                                        // "SpitMoveOutBackAndBalance", () -> { // just a trajectory
                                        //         return this.getTrajectoryCommand("SpitMoveOutBackAndBalance");
                                        // },
                        );
}
