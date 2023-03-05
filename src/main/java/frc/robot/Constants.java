// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import cfutil.CharacterizationConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

        public static class SpeedConstants {
                // TODO: Be aware these numbers were arbitrarily set -- How do we find and fix
                // that?
                public static final double MAX_SPEED_METERS_PER_SECOND = 3.0; // 3 meters per second
                public static final double MAX_ACCELERATION_METERS_PER_SECONDSQ = 3.0; // 3 meters per second squared,
                                                                                       // so 1
                                                                                       // second to full speed
                public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 51; // 51 radians per seconcd
                public static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECONDSQ = MAX_ANGULAR_SPEED_RADIANS_PER_SECOND
                                / (0.40); // 51 rad per second, in 400 milliseconds
        }

        public static class DrivetrainConstants {
                public static final double WHEEL_RADIUS = Units.inchesToMeters(2.0); // meters // MK4 Billet wheel has 4
                                                                                     // inches outer diameter, so 2
                                                                                     // inches radius
                public static final double WHEEL_CIRCUMFERENCE = 2 * Math.PI * WHEEL_RADIUS; // meters
                public static final double MK4_DRIVE_MOTOR_GEAR_RATIO = 6.23 / 1.0;//6.75 / 1.0; //8.14 / 1.0;
                public static final double MK4_TURN_MOTOR_GEAR_RATIO = 12.8 / 1.0;

                // Locations of swerve modules, in meters, from the center of the robot
                // Updated for new chassis
                // Distance between right and left wheels, in m
                public static final double DT_TRACK_WIDTH = Units.inchesToMeters(23.5); // 0.5969 meters
                // Distance between front and back wheels, in m
                public static final double DT_WHEEL_BASE = Units.inchesToMeters(23.5);

                public static final Translation2d locationFL = new Translation2d(DT_WHEEL_BASE / 2,
                                DT_TRACK_WIDTH / 2);
                public static final Translation2d locationFR = new Translation2d(DT_WHEEL_BASE / 2,
                                -DT_TRACK_WIDTH / 2);
                public static final Translation2d locationBL = new Translation2d(-DT_WHEEL_BASE / 2,
                                DT_TRACK_WIDTH / 2);
                public static final Translation2d locationBR = new Translation2d(-DT_WHEEL_BASE / 2,
                                -DT_TRACK_WIDTH / 2);

                public static final SwerveDriveKinematics DT_KINEMATICS = new SwerveDriveKinematics(locationFL,
                                locationFR, locationBL, locationBR);

                private static SwerveModuleState frontLeftState = new SwerveModuleState(0.001, Rotation2d.fromDegrees(-45));
                private static SwerveModuleState frontRightState = new SwerveModuleState(0.001, Rotation2d.fromDegrees(45));
                private static SwerveModuleState backLeftState = new SwerveModuleState(0.001, Rotation2d.fromDegrees(45));
                private static SwerveModuleState backRightState = new SwerveModuleState(0.001, Rotation2d.fromDegrees(-45));
                public static final SwerveModuleState[] STATE_BRAKE = { frontLeftState, frontRightState, backLeftState, backRightState };
        }

        public static class MotorConstants {
                // Offset values must be positive!

                // Module 4
                public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 7;
                public static final int FRONT_LEFT_TURN_MOTOR_ID = 8;
                public static final int FRONT_LEFT_ABS_ENCODER_CHANNEL = 4;
                public static final double FRONT_LEFT_ABS_ENCODER_OFFSET_ROTATIONS = 0.290; // 1.302;// 2.794;
                public static final CharacterizationConstants FRONT_LEFT_DRIVE_MOTOR_CHARACTERIZATION_CONSTANTS = new CharacterizationConstants.Builder()
                                .setFeedforwardConstants(0.60709, 2.7646, 0.15464)
                                .setFeedbackConstants(2.64, 0.0, 0.0)
                                .build();
                public static final CharacterizationConstants FRONT_LEFT_TURN_MOTOR_CHARACTERIZATION_CONSTANTS = new CharacterizationConstants.Builder()
                                // .setFeedforwardConstants(0.12178, 0.22138, 0.0029061) // 0.267, 1.3978, 0.021241
                                .setFeedbackConstants(5.7583, 0.0, 0.05) // 0.38743, 0, 0
                                .build();
                // .setFeedforwardConstants(0.036935, 0.22318, 0.013196)
                // .setFeedbackConstants(6.0656, 0.0, 0.3549)
                // .build();

                // Module 3
                public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 5;
                public static final int FRONT_RIGHT_TURN_MOTOR_ID = 6;
                public static final int FRONT_RIGHT_ABS_ENCODER_CHANNEL = 3;
                public static final double FRONT_RIGHT_ABS_ENCODER_OFFSET_ROTATIONS = 0.360; // 2.362;// 1.857;
                public static final CharacterizationConstants FRONT_RIGHT_DRIVE_MOTOR_CHARACTERIZATION_CONSTANTS = new CharacterizationConstants.Builder()
                                .setFeedforwardConstants(0.49831, 2.6069, 0.13338)
                                .setFeedbackConstants(2.5147, 0.0, 0.0)
                                .build();
                public static final CharacterizationConstants FRONT_RIGHT_TURN_MOTOR_CHARACTERIZATION_CONSTANTS = new CharacterizationConstants.Builder()
                                // .setFeedforwardConstants(0.12178, 0.22138, 0.0029061) // 0.24355, 1.3793, 0.017404
                                .setFeedbackConstants(5.7583, 0.0, 0.05) // 0.27857, 0, 0
                                .build();

                // Module 2
                public static final int BACK_LEFT_DRIVE_MOTOR_ID = 3;
                public static final int BACK_LEFT_TURN_MOTOR_ID = 4;
                public static final int BACK_LEFT_ABS_ENCODER_CHANNEL = 2;
                public static final double BACK_LEFT_ABS_ENCODER_OFFSET_ROTATIONS = 0.639; // 2.632;// 1.113;
                public static final CharacterizationConstants BACK_LEFT_DRIVE_MOTOR_CHARACTERIZATION_CONSTANTS = new CharacterizationConstants.Builder()
                                .setFeedforwardConstants(0.60709, 2.7646, 0.15464)
                                .setFeedbackConstants(2.64, 0.0, 0.0)
                                .build();
                public static final CharacterizationConstants BACK_LEFT_TURN_MOTOR_CHARACTERIZATION_CONSTANTS = new CharacterizationConstants.Builder()
                                // .setFeedforwardConstants(0.12178, 0.22138, 0.0029061) // 0.3011, 1.3791, 0.078867
                                .setFeedbackConstants(5.7583, 0.0, 0.05) // 1.3611, 0, 0
                                .build();
                // .setFeedforwardConstants(0.10355, 0.22137, 0.019083)
                // .setFeedbackConstants(6.5677, 0.0, 0.45055)
                // .build();

                // Module 1
                public static final int BACK_RIGHT_DRIVE_MOTOR_ID = 1;
                public static final int BACK_RIGHT_TURN_MOTOR_ID = 2;
                public static final int BACK_RIGHT_ABS_ENCODER_CHANNEL = 1;
                public static final double BACK_RIGHT_ABS_ENCODER_OFFSET_ROTATIONS = 0.43; // 1.432;// 0.928;
                public static final CharacterizationConstants BACK_RIGHT_DRIVE_MOTOR_CHARACTERIZATION_CONSTANTS = new CharacterizationConstants.Builder()
                                .setFeedforwardConstants(0.49831, 2.6069, 0.13338)
                                .setFeedbackConstants(2.5147, 0.0, 0.0) // 2.3645
                                .build();
                public static final CharacterizationConstants BACK_RIGHT_TURN_MOTOR_CHARACTERIZATION_CONSTANTS = new CharacterizationConstants.Builder()
                                // .setFeedforwardConstants(0.12178, 0.22138, 0.0029061) // 0.17362, 1.3895, 0.10071
                                .setFeedbackConstants(5.7583, 0.0, 0.05) // 1.5034, 0, 0
                                .build();
                // .setFeedforwardConstants(0.082718, 0.22418, 0.0087416)
                // .setFeedbackConstants(5.3903, 0.0, 0.26148)
                // .build();

                // public static final CharacterizationConstants
                // DRIVE_MOTOR_CHARACTERIZATION_CONSTANTS = new
                // CharacterizationConstants.Builder()
                // .setFeedforwardConstants(0.67366, 2.9808, 0.088989)
                // .setFeedbackConstants(1.8428, 0.0, 0.0)
                // .build();

                // .setFeedforwardConstants(0.32, 1.51, 0.27)
                // .setFeedbackConstants(0.05, 0.0, 0.0)
                // .build();

                // TODO: These turn motor constants are right for one motor, check for all
                // motors
                // public static final CharacterizationConstants
                // FRONT_LEFT_TURN_MOTOR_CHARACTERIZATION_CONSTANTS = new
                // CharacterizationConstants.Builder()
                // .setFeedforwardConstants(0.70364, 0.21449, 0.0049559)
                // .setFeedbackConstants(4.3347, 0.0, 0.15614)
                // .build();
        }

        public static final class AutoConstants {
                public static final double AUTO_MAX_SPEED_METERS_PER_SECOND = 1.5; // m/s
                public static final double AUTO_MAX_ACCELERATION_METERS_PER_SECONDSQ = 1.5; // m/s/s
                public static final double AUTO_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 10.0; // rad/s
                public static final double AUTO_MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECONDSQ = 10.0; // rad/s/s

                // TODO: Tune these auto controller constants -- In Progress
                public static final double AUTO_XCONTROLLER_KP = 5;
                public static final double AUTO_YCONTROLLER_KP = 5;
                public static final double AUTO_THETACONTROLLER_KP = 2;

                public static final TrapezoidProfile.Constraints AUTO_THETACONTROLLER_CONSTRAINTS = new TrapezoidProfile.Constraints(
                                AUTO_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND,
                                AUTO_MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECONDSQ);

                public static final TrajectoryConfig AUTO_TRAJECTORY_CONFIG = new TrajectoryConfig(
                                AUTO_MAX_SPEED_METERS_PER_SECOND,
                                AUTO_MAX_ACCELERATION_METERS_PER_SECONDSQ)
                                .setKinematics(DrivetrainConstants.DT_KINEMATICS);
        }

        // OI means Operater Interface :D
        public static class OIConstants {
                public static final int DRIVER_CONTROLLER_PORT = 0;
                public static final int SUBSYSTEM_CONTROLLER_PORT = 1;
                public static final double JOYSTICK_DEADBAND = 0.08;

                public static double SPEED_LIMIT_XY = 0.75;
                public static double SPEED_LIMIT_TURN = 0.15;
        }

        public static class IntakeConstants {
                public static final int INTAKE_MOTOR_ID = 13;
                public static final int CONVEYOR_MOTOR_ID = 9;

                public static final double INTAKE_IN_SPEED = -.5;
                public static final double INTAKE_OUT_SPEED = .5;
                public static final double CONVEYOR_IN_SPEED = .3;
                public static final double CONVEYOR_OUT_SPEED = -.3;
        }

}
