package frc.robot.subsystems.intakePivot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

public class IntakePivot {
        public IntakePivot(){
        configureMotors();
    }

     TalonFX intakePivotMotor = new TalonFX(IntakePivotConstants.kIntakePivotMotorID);

    //Current Limits
    private CurrentLimitsConfigs currentLimits=
      new CurrentLimitsConfigs()
        .withSupplyCurrentLimit(IntakePivotConstants.kSupplyCurrentLimit)
        .withSupplyCurrentLimitEnable(IntakePivotConstants.kSupplyLimitEnabled)
        .withStatorCurrentLimit(IntakePivotConstants.kStatorLimit)
        .withSupplyCurrentLimitEnable(IntakePivotConstants.kStatorLimitEnabled);

    //Inversion and Neutral Mode
    private MotorOutputConfigs motorOutputConfiguration = 
      new MotorOutputConfigs()
        .withInverted(IntakePivotConstants.kInversion)
        .withNeutralMode(IntakePivotConstants.kNeutralMode);

    //Gear Ratio
    private FeedbackConfigs feedbackConfigs =
      new FeedbackConfigs()
      .withSensorToMechanismRatio(IntakePivotConstants.kIntakePivotRatio);

    //Configure the Robot
    private TalonFXConfiguration intakePivotMotorConfiguration =
      new TalonFXConfiguration()
        .withCurrentLimits(currentLimits)
        .withMotorOutput(motorOutputConfiguration)
        .withFeedback(feedbackConfigs);
    
    public void configureMotors(){
        intakePivotMotor.getConfigurator().apply(intakePivotMotorConfiguration);
    }
    
}
