package frc.robot.commands.auto;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
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

        private final PathConstraints CONSTRAINTS_NORMAL = new PathConstraints(2.5, 2.5);
        private final PathConstraints CONSTRAINTS_BALANCE = new PathConstraints(1.5, 1.5);

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

                eventMap.put("print", new PrintCommand("##### PRINT COMMAND TEST"));
        }

        // ===== COMMANDS ===== //
        /**
         * THIS DOESN'T BRING THE ARM BACK DOWN, NEED TO DO
         * new ArmAutoControl(armSubsystem, 0), // Arm bring back down
         */
        // public Command PlaceMidCone() {
        // return new SequentialCommandGroup(
        // new ParallelCommandGroup(
        // new ParallelDeadlineGroup( // Move backwardi to allow arm
        // new WaitCommand(0.85),
        // new MoveStraightCmd(this.swerveSubsystem,
        // MoveDirection.BACKWARD)),
        // new SequentialCommandGroup(
        // new WaitCommand(0.5),
        // // Arm raise to mid cone
        // new ArmAutoControl(armSubsystem, 1))),
        // new ParallelDeadlineGroup( // Move forward for 0.5 seconds
        // new WaitCommand(0.85),
        // new MoveStraightCmd(swerveSubsystem, MoveDirection.FORWARD)),
        // new ParallelCommandGroup(
        // new ArmAutoControl(armSubsystem, 0),
        // new SequentialCommandGroup(
        // new WaitCommand(1),
        // new ParallelDeadlineGroup( // Move backwardi to allow
        // // arm
        // new WaitCommand(0.75),
        // new MoveStraightCmd(
        // this.swerveSubsystem,
        // MoveDirection.BACKWARD)))));
        // }

        /**
         * @param direction "Left" or "Right"
         * @return
         */
        // public Command PlaceConeGrabCubeAndSpit(String direction) {
        // return new SequentialCommandGroup(
        // this.PlaceMidCone(), // Place the cone
        // new ParallelDeadlineGroup(
        // this.getTrajectoryCommand(
        // direction + "PlaceConeMoveOutGrabNewBlockMoveBackPart1",
        // CONSTRAINTS_NORMAL),
        // new ArmAutoControl(armSubsystem, 0), // Arm bring back down
        // new IntakeCommand(intakeSubsystem, IntakeDirection.IN)),
        // // new ParallelDeadlineGroup( SEE IF PATH CAN DO IT FINE
        // // new WaitCommand(1.7),
        // // new MoveStraightCmd(swerveSubsystem, MoveDirection.BACKWARD),
        // // new IntakeCommand(intakeSubsystem, IntakeDirection.IN)),
        // this.getTrajectoryCommand(direction +
        // "PlaceConeMoveOutGrabNewBlockMoveBackPart2",
        // CONSTRAINTS_NORMAL),
        // // Runs intake for 0.5 seconds
        // new ParallelDeadlineGroup(
        // new WaitCommand(1),
        // new IntakeCommand(intakeSubsystem, IntakeDirection.OUT)));
        // };

        // public Command PlaceConeAndBalance() {
        // return new SequentialCommandGroup(
        // this.PlaceMidCone(), // Place the cone
        // new ParallelDeadlineGroup(
        // // Backwards because placing cone first
        // this.getTrajectoryCommand("BackwardsOverStationAndBacktoBalance",
        // CONSTRAINTS_BALANCE),
        // // Arm bring back down
        // new ArmAutoControl(armSubsystem, 0)),
        // new GyroAutocorrectCommand(swerveSubsystem));
        // }

        // Add Place Cone Grab Cube & Balance

        // public Command OverStationAndBacktoBalance() {
        // return new SequentialCommandGroup(
        // this.getTrajectoryCommand("OverStationAndBacktoBalance",
        // CONSTRAINTS_BALANCE),
        // new GyroAutocorrectCommand(swerveSubsystem));
        // };

        public Command SpitCubeGrabCubeAndSpit() {
                return new SequentialCommandGroup(
                                new ParallelDeadlineGroup( // Spit cube
                                                new WaitCommand(0.69),
                                                new IntakeCommand(intakeSubsystem, IntakeDirection.OUT)),
                                this.getTrajectoryCommand("SpitCubeGrabCubeAndSpitPart1", false, CONSTRAINTS_NORMAL),
                                this.getTrajectoryCommand("SpitCubeGrabCubeAndSpitPart2", false, CONSTRAINTS_NORMAL),
                                new ParallelDeadlineGroup( // Spit cube
                                                new WaitCommand(0.69),
                                                new IntakeCommand(intakeSubsystem, IntakeDirection.OUT)));
        };

        public Command SpitCubeAndBalance() {
                return new SequentialCommandGroup(
                                new ParallelDeadlineGroup(
                                                new WaitCommand(1),
                                                new IntakeCommand(intakeSubsystem, IntakeDirection.OUT)),
                                this.getTrajectoryCommand("OverStationAndBacktoBalance", false, CONSTRAINTS_BALANCE),
                                new GyroAutocorrectCommand(swerveSubsystem));
        };

        public Command FarSideSpitAndTaxi() {
                return new SequentialCommandGroup(
                                new ParallelDeadlineGroup(
                                                new WaitCommand(1),
                                                new IntakeCommand(intakeSubsystem, IntakeDirection.OUT)),
                                this.getTrajectoryCommand("FarSideSpitAndTaxi", false, CONSTRAINTS_BALANCE));
        };

        // public Command FarSideConeAndTaxi() {
        // return new SequentialCommandGroup(
        // this.PlaceMidCone(),
        // new ParallelCommandGroup(
        // new ArmAutoControl(armSubsystem, 0), // Arm bring back down
        // this.getTrajectoryCommand("BackwardsFarSideSpitAndTaxi",
        // CONSTRAINTS_BALANCE)));

        // };

        // ===== HELPER ===== //
        public Command getTrajectoryCommand(String fileName, boolean reversed, PathConstraints constraints) {
                // The trajectory to follow
                List<PathPlannerTrajectory> plotTrajectory = Trajectories.loadTrajectory(fileName, reversed,
                                constraints);

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
        public Map<String, Supplier<Command>> AUTO_CMD_MAP = Map.of(
                                        // "PlaceConeAndBalance", () -> {
                                        // return this.PlaceConeAndBalance();
                                        // },
                                        // "LeftPlaceConeGrabCubeAndSpit", () -> {
                                        // return this.PlaceConeGrabCubeAndSpit("Left");
                                        // },
                                        // "RightPlaceConeGrabCubeAndSpit", () -> {
                                        // return this.PlaceConeGrabCubeAndSpit("Right");
                                        // },
                                        "SpitCubeAndBalance", () -> {
                                                return this.SpitCubeAndBalance();
                                        },
                                        "FarSideSpitAndTaxi", () -> {
                                                return this.FarSideSpitAndTaxi();
                                        },
                                        "SpitCubeGrabCubeAndSpit", () -> {
                                                return this.SpitCubeGrabCubeAndSpit();
                                        //        return this.getTrajectoryCommand("SpitCubeGrabCubeAndSpitPart1", false, CONSTRAINTS_NORMAL);
                                        },
                                        "Straight", () -> {
                                                return this.getTrajectoryCommand("Straight", false, this.CONSTRAINTS_NORMAL);
                                        }
                                        // ,
                                         // "FarSideConeAndTaxi", () -> {
                                         // return this.FarSideConeAndTaxi();
                                         //
                        );
}
