package frc.robot.sparkmaxconfigs;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.RelativeEncoder;


public class SingleMotor {
    public final SparkMax motor;


    public SingleMotor(SparkMax motor, SparkBaseConfig baseConfig, boolean inverse){
        baseConfig.inverted(inverse);

        motor.configure(
                baseConfig,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);

        this.motor = motor;
    }


    public RelativeEncoder getEncoder() {
        return this.motor.getEncoder();
    }


    public void accept(double value) {
        this.motor.set(value);
    }


    public void setVoltage(double value) {
        this.motor.setVoltage(value);
    }


    public void stop() {
        this.motor.stopMotor();
    }
}
