package frc.robot.utils;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElasticDashboard extends SubsystemBase{

    @Override
    public void periodic() {
        Logger.recordOutput("ElasticDashboard/HubActive", Field.isHubActive());

        publishPeriodTime();
        
        
    }

    public void publishPeriodTime() {
        double time = DriverStation.getMatchTime();
        double value = time;

        if (time < 0) {
            time = 0.0;
        }

        Logger.recordOutput("ElasticDashboard/Matchtime", time);

        if (time > 130){
            value = time - 130;
            Logger.recordOutput("ElasticDashboard/Phase", "Transition");
        }
        else if(time > 105) {
            value = time - 105;
            Logger.recordOutput("ElasticDashboard/Phase", "Shift 1");
        }
        else if(time > 80) {
            value = time - 80;
            Logger.recordOutput("ElasticDashboard/Phase", "Shift 2");
        }
        else if(time > 55) {
            value = time - 55;
            Logger.recordOutput("ElasticDashboard/Phase", "Shift 3");
        }
        else if(time > 30) {
            if (Field.isHubActive()) {
                value = time;
            }
            else {
                value = time - 30;
            }
            
            Logger.recordOutput("ElasticDashboard/Phase", "Shift 4");
        }
        else {
            value = time;
            Logger.recordOutput("ElasticDashboard/Phase", "Auto/End");
        }

        Logger.recordOutput("ElasticDashboard/Shifttime", value);
    }
}