package frc.robot.subsystems.hood;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

public class Hood {

    public Hood(){
        configureMotors();
    }

     TalonFX hoodMotor = new TalonFX(HoodConstants.kHoodMotorID);

    //Current Limits
    private CurrentLimitsConfigs currentLimits=
      new CurrentLimitsConfigs()
        .withSupplyCurrentLimit(HoodConstants.kSupplyCurrentLimit)
        .withSupplyCurrentLimitEnable(HoodConstants.kSupplyLimitEnabled)
        .withStatorCurrentLimit(HoodConstants.kStatorLimit)
        .withSupplyCurrentLimitEnable(HoodConstants.kStatorLimitEnabled);

    //Inversion and Neutral Mode
    private MotorOutputConfigs motorOutputConfiguration = 
      new MotorOutputConfigs()
        .withInverted(HoodConstants.kInversion)
        .withNeutralMode(HoodConstants.kNeutralMode);

    //Gear Ratio
    private FeedbackConfigs feedbackConfigs =
      new FeedbackConfigs()
      .withSensorToMechanismRatio(HoodConstants.kHoodRatio);

    //Configure the Robot
    private TalonFXConfiguration hoodMotorConfiguration =
      new TalonFXConfiguration()
        .withCurrentLimits(currentLimits)
        .withMotorOutput(motorOutputConfiguration)
        .withFeedback(feedbackConfigs);
    
    public void configureMotors(){
        hoodMotor.getConfigurator().apply(hoodMotorConfiguration);
    }
}
