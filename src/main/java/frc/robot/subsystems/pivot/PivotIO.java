package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public interface PivotIO {

    @AutoLog
    public static class PivotIOInputs {

        public Angle pivotPosition;

        public AngularVelocity pivotVelocity;

        public Current pivotStatorCurrent;
        public Current pivotSupplyCurrent;
    }

    public default void updateInputs(PivotIOInputs inputs) {}

    public default void setVelocity(AngularVelocity velocity, int slot) {}

    public default void setPosition(Angle position, int slot) {}

    public default void setMotionMagicPosition(Angle position, int slot) {}

    public default void coast() {}

    public default void configurePID(double extensionP, double extensionI,
        double extensionD, double retractP,
        double retractI, double retractD) {}
    
}
