package frc.robot.commands.pneumatics;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PneumaticsSubsystem;

public class CompressorOnOff extends CommandBase {
    private final PneumaticsSubsystem pneumaticsSubsystem;

    public CompressorOnOff(PneumaticsSubsystem pneumaticsSubsystem) {
        this.pneumaticsSubsystem = pneumaticsSubsystem;
        addRequirements(pneumaticsSubsystem);
    }

    @Override
    public void initialize() {
        // System.out.println("Compressor Status Switch");
        if (pneumaticsSubsystem.statusOfCompressor()) {
            pneumaticsSubsystem.compressorOff();
        }
  
        else {
            pneumaticsSubsystem.compressorOn();
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