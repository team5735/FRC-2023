package frc.robot.commands.pneumatics;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PneumaticsSubsystem;

public class extendRetract extends CommandBase {
    private final PneumaticsSubsystem pneumaticsSubsystem;
    private boolean extended = false;

    public extendRetract(PneumaticsSubsystem pneumaticsSubsystem) {
        this.pneumaticsSubsystem = pneumaticsSubsystem;
        addRequirements(pneumaticsSubsystem);
       
        
    }

    @Override
    public void initialize() {
        if(!extended) {
            pneumaticsSubsystem.extend();
            extended = true;
        }
        else if(extended) {
            pneumaticsSubsystem.retract();
            extended = false;
        }

    }

    @Override
    public void execute() {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}