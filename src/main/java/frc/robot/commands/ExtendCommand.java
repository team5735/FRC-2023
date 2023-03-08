package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ExtenderSubsystem;

public class ExtendCommand extends CommandBase {
    private final ExtenderSubsystem extenderSubsystem;
    private final Supplier<Double> ySpeed;

    public ExtendCommand(ExtenderSubsystem extenderSubsystem, Supplier<Double> ySpeed) {
        this.extenderSubsystem = extenderSubsystem;
        this.ySpeed = ySpeed;
        addRequirements(extenderSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double yInput = Math.pow(ySpeed.get(), 3) * 0.5;
        yInput = Math.abs(yInput) > Constants.OIConstants.JOYSTICK_DEADBAND ? yInput : 0.0;
        this.extenderSubsystem.extenderControl(yInput);
        System.out.println("extending :D");
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}