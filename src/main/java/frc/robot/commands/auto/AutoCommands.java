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
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.trajectories.Trajectories;

public class AutoCommands {

    // The dropdown select options
    public static Map<String, IAutoCommand<SwerveSubsystem, ArmSubsystem, GrabberSubsystem, IntakeSubsystem, Map<String, Command>>> AUTO_CMD_MAP = Map.of(
        "PlaceGrabCubeAndSpit", AutoCommands.PlaceGrabCubeAndSpit
    );

    // ===== COMMANDS ===== //
    public static IAutoCommand<SwerveSubsystem, ArmSubsystem, GrabberSubsystem, IntakeSubsystem, Map<String, Command>> PlaceGrabCubeAndSpit = (
            swerve, arm, grabber, intake, eventMap) -> {
        return new SequentialCommandGroup(
            // Bring arm to straight out
            new ArmAutoControl(arm, 1), // NEEDS ARM TO HAVE SPACE TO MOVE,
            // Runs grabber for 0.5 seconds
            new ParallelDeadlineGroup(
                new WaitCommand(0.5), 
                new GrabberCommand(grabber, GrabberDirection.OUT)
            ),
            // Start moving and bring arm back down
            new ParallelCommandGroup(
                new ArmAutoControl(arm, 0),
                // Should move and grab a cube and bring it back
                AutoCommands.getTrajectoryCommand("PlaceGrabCubeAndSpit", swerve, eventMap)
            ),
            // Runs intake for 0.5 seconds
            new ParallelDeadlineGroup(
                new WaitCommand(0.5), 
                new IntakeCommand(intake, IntakeDirection.OUT)
            )
        );
    };

    // A simple "run certain trajectory" command
    public static IAutoCommand<SwerveSubsystem, ArmSubsystem, GrabberSubsystem, IntakeSubsystem, Map<String, Command>> SpitMoveOutBackAndBalance = (
            swerve, arm, grabber, intake, eventMap) -> {
        return AutoCommands.getTrajectoryCommand("SpitMoveOutBackAndBalance", swerve, eventMap);
    };

    // ===== HELPER ===== //
    public static Command getTrajectoryCommand(String fileName, SwerveSubsystem swerveSubsystem,
            Map<String, Command> eventMap) {
        // The trajectory to follow
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
                eventMap,
                true,
                swerveSubsystem);

        return autoBuilder.fullAuto(plotTrajectory);
    }
}
