package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public interface IndexerIO {

    @AutoLog
    public static class IndexerIOInputs {

        public AngularVelocity indexerSpinVelocity;

        public Current statorCurrent;
        public Current supplyCurrent;
    }
    public default void updateInputs(IndexerIOInputs inputs) {}

    public default void setVelocity(AngularVelocity velocity) {}
    
}

