package frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/*
    Robot Reference Frame
  
    +Rotation: Clockwise (Rotate Left)

             +X 
         ___________
        |     F     |
        |           |
   +Y   |L         R|    -Y
        |           |
        |_____B_____|

             -X
 */

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft, frontRight, backLeft, backRight;
    private final AHRS gyro;
    private final SwerveDriveOdometry odometer;
    private final Field2d m_field = new Field2d();

    private boolean isFieldOrientedEnabled = false;

    public SwerveSubsystem() {

        this.frontLeft = new SwerveModule(
                Constants.MotorConstants.FRONT_LEFT_DRIVE_MOTOR_ID,
                Constants.MotorConstants.FRONT_LEFT_TURN_MOTOR_ID,
                false, // drive motor reversed
                false, // turn motor reversed
                Constants.MotorConstants.FRONT_LEFT_ABS_ENCODER_CHANNEL,
                Constants.MotorConstants.FRONT_LEFT_ABS_ENCODER_OFFSET_ROTATIONS,
                1,
                Constants.MotorConstants.FRONT_LEFT_DRIVE_MOTOR_CHARACTERIZATION_CONSTANTS,
                Constants.MotorConstants.FRONT_LEFT_TURN_MOTOR_CHARACTERIZATION_CONSTANTS);

        this.frontRight = new SwerveModule(
                Constants.MotorConstants.FRONT_RIGHT_DRIVE_MOTOR_ID,
                Constants.MotorConstants.FRONT_RIGHT_TURN_MOTOR_ID,
                false, // drive motor reversed
                false, // turn motor reversed
                Constants.MotorConstants.FRONT_RIGHT_ABS_ENCODER_CHANNEL,
                Constants.MotorConstants.FRONT_RIGHT_ABS_ENCODER_OFFSET_ROTATIONS,
                2,
                Constants.MotorConstants.FRONT_RIGHT_DRIVE_MOTOR_CHARACTERIZATION_CONSTANTS,
                Constants.MotorConstants.FRONT_RIGHT_TURN_MOTOR_CHARACTERIZATION_CONSTANTS);

        this.backLeft = new SwerveModule(
                Constants.MotorConstants.BACK_LEFT_DRIVE_MOTOR_ID,
                Constants.MotorConstants.BACK_LEFT_TURN_MOTOR_ID,
                false, // drive motor reversed
                false, // turn motor reversed
                Constants.MotorConstants.BACK_LEFT_ABS_ENCODER_CHANNEL,
                Constants.MotorConstants.BACK_LEFT_ABS_ENCODER_OFFSET_ROTATIONS,
                3,
                Constants.MotorConstants.BACK_LEFT_DRIVE_MOTOR_CHARACTERIZATION_CONSTANTS,
                Constants.MotorConstants.BACK_LEFT_TURN_MOTOR_CHARACTERIZATION_CONSTANTS);

        this.backRight = new SwerveModule(
                Constants.MotorConstants.BACK_RIGHT_DRIVE_MOTOR_ID,
                Constants.MotorConstants.BACK_RIGHT_TURN_MOTOR_ID,
                false, // drive motor reversed
                false, // turn motor reversed
                Constants.MotorConstants.BACK_RIGHT_ABS_ENCODER_CHANNEL,
                Constants.MotorConstants.BACK_RIGHT_ABS_ENCODER_OFFSET_ROTATIONS,
                4,
                Constants.MotorConstants.BACK_RIGHT_DRIVE_MOTOR_CHARACTERIZATION_CONSTANTS,
                Constants.MotorConstants.BACK_RIGHT_TURN_MOTOR_CHARACTERIZATION_CONSTANTS);

        this.gyro = new AHRS(SPI.Port.kMXP);
        this.odometer = new SwerveDriveOdometry(
                Constants.DrivetrainConstants.DT_KINEMATICS, // Give the odometry the kinematics of the robot,
                new Rotation2d(0), // and the starting angle of the robot
                new SwerveModulePosition[] {
                        this.frontLeft.getPosition(),
                        this.frontRight.getPosition(),
                        this.backLeft.getPosition(),
                        this.backRight.getPosition()
                }, new Pose2d(0, 0, new Rotation2d())); // Initial position

        // Reset the heading of the robot after 1 second
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                this.zeroHeading();
            } catch (Exception e) {
                throw new RuntimeException("Could not reset the heading of the robot!");
            }
        }).start();
    }

    /**
     * Toggles the field-oriented drive mode on and off
     */
    public void toggleFieldOriented() {
        this.isFieldOrientedEnabled = !this.isFieldOrientedEnabled;
    }

    /**
     * Returns if field-oriented drive is enabled
     * 
     * @return true if field-oriented drive is enabled, false otherwise
     */
    public boolean isFieldOrientedEnabled() {
        return this.isFieldOrientedEnabled;
    }

    /**
     * Resets the heading of the robot.
     */
    public void zeroHeading() {
        System.out.println("Resetting gyro heading to 0");
        this.gyro.reset();
    }

    /**
     * Gets the current heading of the gyro, in degrees
     * 
     * @return
     */
    public double getGyroHeadingDeg() {
        // Negative because NavX says clockwise is positive but WPILib says
        // counterclockwise is positive
        return Math.IEEEremainder(-this.gyro.getAngle(), 360);
    }

    /**
     * Gets the pitch of the gyro, in radians, for autobalancing purposes.
     */
    public double getGyroPitchRad() {
        return Units.degreesToRadians(this.gyro.getPitch());
    }

    /**
     * Constructs a Rotation2d object from the gyro heading, converting it to
     * radians
     * 
     * @return the Rotation2d object
     */
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(this.getGyroHeadingDeg());
    }

    /**
     * Gets the current position of the robot on the field
     * 
     * @return the current position of the robot
     */
    public Pose2d getPose() {
        return this.odometer.getPoseMeters();
    }

    /**
     * Reset the position and heading of the robot to a known location and
     * orientation
     * 
     * @param pose the new position and orientation of the robot
     */
    public void resetOdometry(Pose2d initialPose) {
        // this.frontLeft.resetEncoders();
        // this.frontRight.resetEncoders();
        // this.backLeft.resetEncoders();
        // this.backRight.resetEncoders();
        // this.gyro.reset();
        this.odometer.resetPosition(
                this.gyro.getRotation2d(),
                // new Rotation2d(),
                new SwerveModulePosition[] {
                    this.frontLeft.getPosition(),
                    this.frontRight.getPosition(),
                    this.backLeft.getPosition(),
                    this.backRight.getPosition()
                        // new SwerveModulePosition(0, this.frontLeft.getTurnMotorRotation2d()),
                        // new SwerveModulePosition(0, this.frontRight.getTurnMotorRotation2d()),
                        // new SwerveModulePosition(0, this.backLeft.getTurnMotorRotation2d()),
                        // new SwerveModulePosition(0, this.backRight.getTurnMotorRotation2d())
                },
                initialPose);
    }

    /**
     * Testing purposes. Turns a motor full power to measure velocity
     */
    public void turnFullPower() {
        this.frontRight.turnFullPower();
    }

    /**
     * Drive with open loop control (does not correct for error)
     * @param chassisSpeeds
     */
    public void openLoopDrive(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] moduleStates = Constants.DrivetrainConstants.DT_KINEMATICS
                .toSwerveModuleStates(chassisSpeeds);

        this.setModuleStates(moduleStates, true);
    }

    /**
     * Sets the desired velocity and angle of all swerve modules. Uses FF + PID control.
     * 
     * @param desiredStates
     */
    public void setModuleStates(SwerveModuleState[] desiredStates, boolean isOpenLoop) {
        // Scales all desired states to the maximum speed of the robot
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates,
                Constants.SpeedConstants.MAX_SPEED_METERS_PER_SECOND);

        frontLeft.setDesiredState(desiredStates[0], isOpenLoop);
        frontRight.setDesiredState(desiredStates[1], isOpenLoop);
        backLeft.setDesiredState(desiredStates[2], isOpenLoop);
        backRight.setDesiredState(desiredStates[3], isOpenLoop);
    }

    /**
     * The subsystem method called every 20ms. Updates the odometry of the robot.
     */
    @Override
    public void periodic() {
        odometer.update( // Updates the current angle and position of the robot
                this.getRotation2d(),
                new SwerveModulePosition[] {
                        this.frontLeft.getPosition(), this.frontRight.getPosition(),
                        this.backLeft.getPosition(), this.backRight.getPosition()
                });

        m_field.setRobotPose(this.odometer.getPoseMeters());

        SmartDashboard.putData("Field", m_field);
        SmartDashboard.putString("Odometry", this.odometer.getPoseMeters().toString());

        SmartDashboard.putNumber("FL Drive Pos", this.frontLeft.getDriveWheelPosition());
        SmartDashboard.putNumber("FR Drive Pos", this.frontRight.getDriveWheelPosition());
        SmartDashboard.putNumber("BL Drive Pos", this.backLeft.getDriveWheelPosition());
        SmartDashboard.putNumber("BR Drive Pos", this.backRight.getDriveWheelPosition());


        // SmartDashboard.putString("Odometry",
        // this.odometer.getPoseMeters().toString());

        SmartDashboard.putBoolean("Gyro Calibrating", this.gyro.isCalibrating());
        SmartDashboard.putBoolean("Gyro Connected", this.gyro.isConnected());
        SmartDashboard.putNumber("Robot Heading (deg)", this.getGyroHeadingDeg());
        SmartDashboard.putString("Robot Location (m)", this.getPose().getTranslation().toString());
        SmartDashboard.putBoolean("Field Oriented Enabled", this.isFieldOrientedEnabled);

        SmartDashboard.putNumber("FL Wheel Angle", Units.radiansToDegrees(frontLeft.getTurnMotorAngle()));
        SmartDashboard.putNumber("FR Wheel Angle", Units.radiansToDegrees(frontRight.getTurnMotorAngle()));
        SmartDashboard.putNumber("BL Wheel Angle", Units.radiansToDegrees(backLeft.getTurnMotorAngle()));
        SmartDashboard.putNumber("BR Wheel Angle", Units.radiansToDegrees(backRight.getTurnMotorAngle()));

        // SmartDashboard.putNumber("FL Absolute Encoder", frontLeft.getTurnMotorInRotations());
        // SmartDashboard.putNumber("FR Absolute Encoder", frontRight.getTurnMotorInRotations());
        // SmartDashboard.putNumber("BL Absolute Encoder", backLeft.getTurnMotorInRotations());
        // SmartDashboard.putNumber("BR Absolute Encoder", backRight.getTurnMotorInRotations());

        // Doesn't actually do what its supposed to do
        // SmartDashboard.putNumber("FL Wheel Speed",
        // Units.radiansToDegrees(frontLeft.getTurnMotorAngle()));
        // SmartDashboard.putNumber("FR Wheel Speed",
        // Units.radiansToDegrees(frontRight.getTurnMotorAngle()));
        // SmartDashboard.putNumber("BL Wheel Speed",
        // Units.radiansToDegrees(backLeft.getTurnMotorAngle()));
        // SmartDashboard.putNumber("BR Wheel Speed",
        // Units.radiansToDegrees(backRight.getTurnMotorAngle()));

        // Returns in Degrees -- Pitch and Roll subject to change depending upon gyro orientation
        SmartDashboard.putNumber("Gyro Pitch", this.gyro.getPitch()); // Returns rotation on axis perpendicular to battery
        // SmartDashboard.putNumber("Gyro Roll radians", Units.degreesToRadians(this.gyro.getPitch())); // Returns rotation on the axis of the battery in Radians
        // SmartDashboard.putNumber("Gyro Roll", this.gyro.getRoll()); // Returns rotation on the axis of the battery
        // SmartDashboard.putNumber("Gyro Roll radians", Units.degreesToRadians(this.gyro.getRoll())); // Returns rotation on the axis of the battery in Radians
        // SmartDashboard.putNumber("Gyro Yaw", this.gyro.getYaw()); // Flat Axis -- Rotation

    }

    /**
     * Stops all swerve modules
     */
    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    /**
     * Set the swerve drive to brake position, which is 45 degrees
     */
    public void brake() {
        this.setModuleStates(Constants.DrivetrainConstants.STATE_BRAKE, false);
    }
}
