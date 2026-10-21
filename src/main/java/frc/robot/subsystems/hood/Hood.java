package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;

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

    @Logged(name = "hoodPitch")
    public Angle getAngle(){
      Angle angle = hoodMotor.getPosition().getValue();
      return angle;
    }

    @Logged(name = "atTargetAngle")
    public boolean atTragetAngle(){
      return Math.abs(HoodConstants.kTargetAngle - getAngle().in(Degrees)) <= 1.5;

    }
}
