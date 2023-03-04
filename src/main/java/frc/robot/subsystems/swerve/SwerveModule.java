package frc.robot.subsystems.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import cfutil.CharacterizationConstants;
import cfutil.UnitConversion;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class SwerveModule {

    private final WPI_TalonFX driveMotor;
    private final WPI_TalonFX turnMotor;

    private final DutyCycleEncoder turnAbsoluteEncoder;
    private final double absoluteEncoderOffsetRotations;
    private final int moduleId;

    private final SimpleMotorFeedforward driveFF;
    private final PIDController drivePID;

    // Use this if plain PID isn't good enough
    // private final SimpleMotorFeedforward turnFF;
    // private final ProfiledPIDController turnPID;

    private final PIDController turnPID;

    public SwerveModule(int driveMotorId, int turnMotorId, boolean driveMotorReversed, boolean turnMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffsetRotations, int moduleId,
            CharacterizationConstants drivePid,
            CharacterizationConstants turnPid) {

        this.turnAbsoluteEncoder = new DutyCycleEncoder(absoluteEncoderId);
        this.absoluteEncoderOffsetRotations = absoluteEncoderOffsetRotations;

        this.driveMotor = new WPI_TalonFX(driveMotorId);
        this.turnMotor = new WPI_TalonFX(turnMotorId);
        this.driveMotor.setInverted(driveMotorReversed);
        this.turnMotor.setInverted(turnMotorReversed);

        this.driveFF = new SimpleMotorFeedforward(drivePid.getS(), drivePid.getV(), drivePid.getA());
        this.drivePID = new PIDController(drivePid.getP(), drivePid.getI(), drivePid.getD());

        // this.turnFF = new SimpleMotorFeedforward(turnPid.getS(), turnPid.getV(),
        // turnPid.getA());
        // this.turnPID = new ProfiledPIDController(turnPid.getP(), turnPid.getI(),
        // turnPid.getD(),
        // new TrapezoidProfile.Constraints(
        // Constants.SpeedConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND,
        // Constants.SpeedConstants.MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECONDSQ));

        this.turnPID = new PIDController(turnPid.getP(), turnPid.getI(),
                turnPid.getD());
        this.turnPID.enableContinuousInput(-Math.PI, Math.PI);

        this.moduleId = moduleId;

        resetEncoders();
    }

    /**
     * Returns the drive wheel's position in meters
     */
    public double getDriveWheelPosition() {
        return UnitConversion.falconToMeters(
                this.driveMotor.getSelectedSensorPosition());
    }

    /**
     * Returns how fast the drive wheel is turning, in meters per second
     * 
     * @return the velocity of the wheel in meters per second
     */
    public double getDriveWheelVelocity() {
        return UnitConversion.falconToMetersPerSecond(
                this.driveMotor.getSelectedSensorVelocity());
    }

    /**
     * Returns the current angle the wheel is pointed at, in radians
     * 
     * @return the current angle the wheel is pointed at, in radians
     */
    public double getTurnMotorAngle() {
        return UnitConversion.absoluteEncoderToRadians(
                this.turnAbsoluteEncoder.get(), // in units of rotations
                this.absoluteEncoderOffsetRotations); // offset
    }

    public Rotation2d getTurnMotorRotation2d() {
        return new Rotation2d(this.getTurnMotorAngle());
    }

    public double getTurnMotorInRotations() {
        return this.turnAbsoluteEncoder.get();
    }

    /**
     * Returns how fast the wheel turn motor is spinning in rad/s
     */
    public double getTurnMotorVelocity() {
        return UnitConversion.falconToRadPerSec(
                this.turnMotor.getSelectedSensorVelocity());
    }

    public void resetEncoders() {
        this.driveMotor.setSelectedSensorPosition(0);

        // Unnecessary because we will use absolute encoder to get angle instead of
        // Falcon
        // turningEncoder.setPosition(getAbsoluteEncoderRad());
    }

    /**
     * Returns the current state of the swerve module (velocity and angle of wheel)
     * 
     * @return the current state of the swerve module (velocity and angle of wheel)
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
                this.getDriveWheelVelocity(),
                this.getTurnMotorRotation2d());
    }

    /**
     * Returns the current position (drive motor position and turn motor angle)
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                this.getDriveWheelPosition(),
                this.getTurnMotorRotation2d());
    }

    public void driveFullPower() {
        this.driveMotor.set(ControlMode.PercentOutput, 1.0);
        SmartDashboard.putNumber("Swerve[" + this.moduleId + "] Drive Motor Speed", this.getDriveWheelVelocity());
    }

    public void turnFullPower() {
        this.turnMotor.set(ControlMode.PercentOutput, 1.0);
        System.out.println(this.getTurnMotorVelocity());
    }

    /**
     * Set the desired state of the swerve module (velocity and angle of wheel)
     * 
     * @param state the desired state of the swerve module (velocity and angle of
     *              wheel)
     */
    public void setDesiredState(SwerveModuleState state) {
        // If the speed is very small, stop motors and do nothing
        if (Math.abs(state.speedMetersPerSecond) < 0.0005) {
            stop();
            return;
        }

        // Optimize the state to minimize the angle change
        state = SwerveModuleState.optimize(state, this.getTurnMotorRotation2d());

        // Set drive motor voltage
        // FeedForward: Input is velocity setpoint, output is voltage
        double driveVoltage = this.driveFF.calculate(state.speedMetersPerSecond);
        // PID: Input is current velocity and velocity setpoint (to calculate error),
        // output is voltage
        driveVoltage += this.drivePID.calculate(this.getDriveWheelVelocity(), state.speedMetersPerSecond);
        // Set the voltage
        this.driveMotor.setVoltage(driveVoltage);

        // Set turn motor voltage
        // WHEN USING PROFILEDPIDCONTROLLER:
        // - FIRST calculate PID: Input is current angle, angle setpoint (to calculate
        // error), output is voltage
        // - FeedForward: Input is current velocity, next velocity (determined by PID
        // controller setpoint), and time diff for acceleration calc
        // double turnVoltage = this.turnPID.calculate(this.getTurnMotorAngle(),
        // state.angle.getRadians());
        // turnVoltage += this.turnFF.calculate(
        // this.getTurnMotorVelocity(), // current TURN MOTOR velocity
        // this.turnPID.getSetpoint().velocity, // next setpoint velocity
        // 0.02); // 20ms is the time diff between each loop
        // // Set the voltage
        // this.turnMotor.setVoltage(turnVoltage);

        // WHEN USING PIDCONTROLLER:
        // - Just use PID: Input is current angle and angle setpoint (to calculate
        // error), output is voltage
        double turnVoltage = this.turnPID.calculate(this.getTurnMotorAngle(),
                state.angle.getRadians());
        // Set the voltage
        this.turnMotor.setVoltage(turnVoltage);

        // Logging
        SmartDashboard.putString("Swerve[" + this.moduleId + "] state", state.toString());
        SmartDashboard.putNumber("Swerve[" + this.moduleId + "] Abs Encoder",
                this.turnAbsoluteEncoder.get());
    }

    /**
     * Stop all motors.
     */
    public void stop() {
        this.driveMotor.set(0);
        this.turnMotor.set(0);
    }
}
