package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class PneumaticsSubsystem extends SubsystemBase {

    private DoubleSolenoid solenoid;
    private boolean pistonExtended;

    private Compressor pcmCompressor;

    public PneumaticsSubsystem() {
        this.solenoid = new DoubleSolenoid(6, PneumaticsModuleType.CTREPCM, 1, 0);
        this.pcmCompressor = new Compressor(6, PneumaticsModuleType.CTREPCM);
        this.pcmCompressor.disable();
    }

    /**
     * Turn the compressor on or off
     */
    public void toggleCompressor() {
        if (this.statusOfCompressor()) {
            this.compressorOff();
        } else {
            this.compressorOn();
        }
    }

    /**
     * Toggle piston
     */
    public void togglePiston() {
        if(!this.pistonExtended) {
            this.pistonExtend();
            this.pistonExtended = true;
        } else {
            this.pistonRetract();
            this.pistonExtended = false;
        }
    }

    private void pistonExtend() {
        this.solenoid.set(Value.kForward);
    }

    private void pistonRetract() {
        this.solenoid.set(Value.kReverse);
    }

    public void stopPiston() {
        this.solenoid.set(Value.kOff);
    }

    /**
     * Returns whether the compressor is on or not.
     */
    private boolean statusOfCompressor() {
        return this.pcmCompressor.isEnabled();
    }

    private void compressorOn() {
        this.pcmCompressor.enableDigital();
    }

    private void compressorOff() {
        this.pcmCompressor.disable();
    }

}
