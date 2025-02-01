package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;


public class KrakenTest {
    private TalonFX motor;
    private TalonFXConfiguration config;
    public KrakenTest() {
        motor = new TalonFX(20);
        config = new TalonFXConfiguration();
        //Add configs here
        motor.getConfigurator().apply(config);

    }
    public void setSpeed(double speed) {
        motor.setControl(new DutyCycleOut(speed));
    }
}
