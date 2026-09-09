package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
    private IndexerIO io;
    private IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

    public Indexer(IndexerIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);
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

    public Command enableCommand() {
        // Runs after setting control to prevent the default (disable) commmand from
        // being called until desired
        return setVelocityCommand(IndexerConstants.ENABLED_VELOCITY);
    }

    public Command setEnabledCommand(Supplier<Boolean> enabledSupplier) {
        return run(() -> {
            boolean enabled = enabledSupplier.get();
            if (enabled) {
                setVelocity(IndexerConstants.ENABLED_VELOCITY);
            } else {
                setVelocity(RotationsPerSecond.of(0));
            }
        });
    }

    public Command disableCommand() {
        return setVelocityCommand(RotationsPerSecond.of(0));
    }
}