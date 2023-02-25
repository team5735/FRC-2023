package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj.PneumaticsBase;
// import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;


public class PneumaticsSubsystem extends SubsystemBase{
    DoubleSolenoid solenoid = new DoubleSolenoid(6, PneumaticsModuleType.CTREPCM, 1, 0);
    Compressor pcmCompressor = new Compressor(6, PneumaticsModuleType.CTREPCM);


    public PneumaticsSubsystem(){
        
    }
    @Override
     public void periodic() {
    // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    }
    public void extend() {
        System.out.println("Extend");
        solenoid.set(Value.kForward);
    }
    public void retract() {
        // System.out.println("retract");

        solenoid.set(Value.kReverse);
    }
    public void off() {
        System.out.println("off");

        solenoid.set(Value.kOff);
    }
    public void compressorOn(){
        System.out.println("compressing");

        pcmCompressor.enableDigital();
    }

    public boolean statusOfCompressor() {
        System.out.println("It's Statusing time!");
        return pcmCompressor.enabled();
    }
    public void compressorOff(){
    //
    
    
    
    System.out.println("compressor off");

        pcmCompressor.disable();
    }
    
}
