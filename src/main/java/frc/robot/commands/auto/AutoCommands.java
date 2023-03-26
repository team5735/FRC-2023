package frc.robot.commands.auto;

import java.util.HashMap;
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
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.intake.IntakeCommand.IntakeDirection;
import frc.robot.commands.swerve.BrakeCommand;
import frc.robot.commands.swerve.GyroAutocorrectCommand;
import frc.robot.commands.swerve.MoveStraightCmd;
import frc.robot.commands.swerve.MoveStraightCmd.MoveDirection;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.trajectories.Trajectories;

public class AutoCommands {

        private final SwerveSubsystem swerveSubsystem;
        private final IntakeSubsystem intakeSubsystem;

        private final PathConstraints CONSTRAINTS_NORMAL, CONSTRAINTS_BALANCE;

        public Map<String, Command> eventMap = new HashMap<>();

        public AutoCommands(SwerveSubsystem swerveSubsystem, IntakeSubsystem intakeSubsystem) {
                this.swerveSubsystem = swerveSubsystem;
                this.intakeSubsystem = intakeSubsystem;

                this.CONSTRAINTS_NORMAL = new PathConstraints(2.0, 2.0);
                this.CONSTRAINTS_BALANCE = new PathConstraints(1.5, 1.5);

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
        // Add Place Cone Grab Cube & Balance

        public Command OverStationAndBacktoBalance() {
                return new SequentialCommandGroup(
                                this.getTrajectoryCommand("OverStationAndBacktoBalance", CONSTRAINTS_BALANCE),
                                new GyroAutocorrectCommand(swerveSubsystem));
        };

        public Command SpitCubeAndBalance() {
                return new SequentialCommandGroup(
                                new ParallelDeadlineGroup(
                                                new WaitCommand(2),
                                                new IntakeCommand(intakeSubsystem, IntakeDirection.OUT)),
                                this.getTrajectoryCommand("OverStationAndBacktoBalance", CONSTRAINTS_BALANCE),
                                new GyroAutocorrectCommand(swerveSubsystem));
        };

        public Command FarSideSpitAndTaxi() {
                return new SequentialCommandGroup(
                                new ParallelDeadlineGroup(
                                                new WaitCommand(1),
                                                new IntakeCommand(intakeSubsystem, IntakeDirection.OUT)),
                                this.getTrajectoryCommand("FarSideSpitAndTaxi", CONSTRAINTS_BALANCE));
        };

        // ===== HELPER ===== //
        public Command getTrajectoryCommand(String fileName, PathConstraints constraints) {
                // The trajectory to follow
                PathPlannerTrajectory plotTrajectory = Trajectories.loadTrajectory(fileName, constraints);

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
                                        "OverStationAndBacktoBalance", () -> {
                                                return this.OverStationAndBacktoBalance();
                                        },
                                        "FarSideSpitAndTaxi", () -> {
                                                return this.FarSideSpitAndTaxi();
                                        }// ,
                                         // "FarSideConeAndTaxi", () -> {
                                         // return this.FarSideConeAndTaxi();
                                         //
                        );
}
