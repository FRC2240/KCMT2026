package frc.robot.utils;

import java.util.HashSet;
import java.util.Set;
import java.util.function.Consumer;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TunableNumber extends SubsystemBase {
    private double value;
    private final String path;
    private final DoubleEntry entry; // Added missing network entry field
    private final Set<Consumer<Double>> listeners = new HashSet<>();
    
    public TunableNumber(String path, double defaultValue) {
        this.path = path;
        this.value = defaultValue;

        this.entry = NetworkTableInstance.getDefault()
            .getTable("Tuning")
            .getDoubleTopic(path)
            .getEntry(defaultValue);
            
        this.entry.set(defaultValue);
    }

    public double get() {
        return value;
    }

    /**
     * Adds a change listener to the tunable number. Whenever the number is changed
     * over NT, the listener will trigger.
     */
    public void addChangeListener(Consumer<Double> listener) {
        this.listeners.add(listener);
    }

    /**
     * Removes a change listener from the tunable number.
     */
    public void removeChangeListener(Consumer<Double> listener) {
        if (!this.listeners.contains(listener)) {
            System.out.println("WARNING: Change listener removed without being added first on path " + path);
            return;
        }

        this.listeners.remove(listener);
    }

    @Override
    public void periodic() {
        double oldValue = value;
        value = entry.get(value);
        
        if (value != oldValue) {
            for (Consumer<Double> listener : listeners) {
                listener.accept(value);
            }
        }
    }
}
