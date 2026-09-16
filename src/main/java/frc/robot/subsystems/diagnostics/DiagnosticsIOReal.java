package frc.robot.subsystems.diagnostics;

import edu.wpi.first.wpilibj.RobotController;
import frc.robot.subsystems.diagnostics.DiagnosticsIO.DiagnosticsIOInputs;

public class DiagnosticsIOReal implements DiagnosticsIO {
    @Override
    public void updateInputs(DiagnosticsIOInputsAutoLogged inputs) {
        inputs.BatteryVoltage = RobotController.getBatteryVoltage();
    }
}
