package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private ShooterIO io;
    private ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    

    public Shooter(ShooterIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
    }

    public void setVelocity(AngularVelocity velocity) {
        io.setVelocity(velocity);
    }

    // For acually doing stuff
    public Command setVelocityCommand(AngularVelocity velocity) {
        return runOnce(() -> io.setVelocity(velocity));
    }

    // For tuning
    public Command setVelocityCommand(Supplier<AngularVelocity> velocity) {
        return run(() -> io.setVelocity(velocity.get()));
    }

    public Command disableCommand() {
        return setVelocityCommand(RotationsPerSecond.of(0));
    }
    public Command enableCommand() {
        return setVelocityCommand(RotationsPerSecond.of(10));
    }

    public AngularVelocity getVelocity() {
      return inputs.LeftLowerFlywheelVelocity;
    }
}