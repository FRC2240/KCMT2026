package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Feeder extends SubsystemBase {
private FeederIO io;
private FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged ();

 public Feeder(FeederIO io) {
    this.io = io;
    
 }

 @Overide
 public void periodic() {
   io.updateInputs(inputs);
  Logger.processInputs{
    "Feeder", inputs
  };
 }

 public void setVelocity(AngularVelocity velocity) {
    io.setVelocity(velocity);
 }

 public Command setVelocityCommand(Supplier<AngularVelocity> velocity) {
    return runOnce(() -> setVelocity(velocity.get()));
}

public Command enableCommand() {
    return runOnce(() -> setVelocity(FeederConstants.ENABLED_VELOCITY));
}
public Command disableCommand() {
    return runOnce(() -> setVelocity(RotationsPerSecond.of(0)));
    }

}