package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;

public interface FeederIO {
    @AutoLog
    public static class FeederIOInputs {
        public AngularVelocity velocity;
    }
    public default void updateInputs (FeederIOInputs inputs) {

    }
    public default void setVelocity (AngularVelocity velocity) {
        
    }

}
    
