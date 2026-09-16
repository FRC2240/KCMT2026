package frc.robot.subsystems.diagnostics;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Diagnostics extends SubsystemBase {
    double minVoltage = 13;
    private final DiagnosticsIO io;
    private final DiagnosticsIOInputsAutoLogged inputs = new DiagnosticsIOInputsAutoLogged();
    
    public Diagnostics(DiagnosticsIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Diagnostiscs", inputs);

        if (RobotController.getBatteryVoltage() < minVoltage) {minVoltage = RobotController.getBatteryVoltage();}
        
        Logger.recordOutput("ElasticDashboard/Voltage", RobotController.getBatteryVoltage());
        
        Logger.recordOutput("ElasticDashboard/MinimumVoltage", minVoltage);
    }
}