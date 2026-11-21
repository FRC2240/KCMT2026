package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Feeder extends SubsystemBase {
private FeederIO io;
private FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged ();

 public Feeder(FeederIO io) {
    this.io = io;
    
 }

 @Override
 public void periodic() {
  io.updateInputs(inputs);
  Logger.processInputs(
  "Feeder", inputs
  );
 }

 public void setVelocity(double velocity) {
    io.setVelocity(RotationsPerSecond.of(velocity));
 }

 public Command setVelocityCommand(Supplier<AngularVelocity> velocity) {
    return runOnce(() -> setVelocity(velocity.get()));
}

public Command enableCommand() {
    return Commands.runOnce(() -> setVelocity(FeederConstants.ENABLED_VELOCITY));
}
public Command disableCommand() {
    return runOnce(() -> setVelocity(RotationsPerSecond.of(0)));
    }

}