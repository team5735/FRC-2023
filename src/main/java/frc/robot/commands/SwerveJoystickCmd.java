package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class SwerveJoystickCmd extends CommandBase {

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private final Supplier<Boolean> slowMode;

    /**
     * Creates a new SwerveJoystickCmd to drive the robot using an Xbox controller.
     * 
     * @param swerveSubsystem    the swerve subsystem
     * @param xSpdFunction       A percentage of the left joystick on the X axis
     * @param ySpdFunction       A percentage of the left joystick on the Y axis
     * @param turningSpdFunction A percentage of the right joystick on the X axis
     */
    public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
            Supplier<Boolean> slowMode) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.slowMode = slowMode;
        this.xLimiter = new SlewRateLimiter(Constants.SpeedConstants.MAX_SPEED_METERS_PER_SECOND);
        this.yLimiter = new SlewRateLimiter(Constants.SpeedConstants.MAX_SPEED_METERS_PER_SECOND);
        this.turningLimiter = new SlewRateLimiter(Constants.SpeedConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // 1. Get real-time joystick inputs
        double xSpeed = Math.pow(xSpdFunction.get(), 3) * 0.5;
        double ySpeed = Math.pow(ySpdFunction.get(), 3) * 0.5; // xbox controller
        double turningSpeed = Math.copySign(Math.pow(turningSpdFunction.get(), 2), turningSpdFunction.get()) * 0.25;

        boolean slowMode = this.slowMode.get();
        // 2. Apply deadband
        if (slowMode) {
            xSpeed = Math.abs(xSpeed) > Constants.OIConstants.SLOW_MODE_JOYSTICK_DEADBAND ? xSpeed : 0.0;
            ySpeed = Math.abs(ySpeed) > Constants.OIConstants.SLOW_MODE_JOYSTICK_DEADBAND ? ySpeed : 0.0;
            turningSpeed = Math.abs(turningSpeed) > 0.01 ? turningSpeed : 0.0;
        } else {
            xSpeed = Math.abs(xSpeed) > Constants.OIConstants.JOYSTICK_DEADBAND ? xSpeed : 0.0;
            ySpeed = Math.abs(ySpeed) > Constants.OIConstants.JOYSTICK_DEADBAND ? ySpeed : 0.0;
            turningSpeed = Math.abs(turningSpeed) > 0.02 ? turningSpeed : 0.0;
        }

        // 2.5 Limit speed
        xSpeed *= Constants.OIConstants.SPEED_LIMIT_XY;
        ySpeed *= Constants.OIConstants.SPEED_LIMIT_XY;
        turningSpeed *= Constants.OIConstants.SPEED_LIMIT_TURN;

        // 2.75 Limit speed if slow mode

        if (slowMode) {
            xSpeed *= 0.4;
            ySpeed *= 0.4;
            turningSpeed *= 0.4;
        }

        // 3. Make the driving smoother
        xSpeed = xLimiter.calculate(xSpeed) * Constants.SpeedConstants.MAX_SPEED_METERS_PER_SECOND; // returns desired
                                                                                                    // velocity in m/s
        ySpeed = yLimiter.calculate(ySpeed) * Constants.SpeedConstants.MAX_SPEED_METERS_PER_SECOND; // returns desired
                                                                                                    // velocity in m/s
        turningSpeed = turningLimiter.calculate(turningSpeed)
                * Constants.SpeedConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND; // returns desired angular velocity in
                                                                                 // rad/s

        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;
        if (this.swerveSubsystem.isFieldOrientedEnabled()) {
            // Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
        } else {
            // Relative to robot
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        swerveSubsystem.openLoopDrive(chassisSpeeds);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
