package frc.robot.swerve;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.RobotConfig;
import com.reduxrobotics.sensors.canandmag.Canandmag;
import com.reduxrobotics.sensors.canandmag.CanandmagSettings;

import edu.wpi.first.units.measure.AngularVelocity;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.io.IOException;

public class Swerve {

    public static final double PI2 = 2.0 * Math.PI;

    public static class Constants {
        // BOT SWITCHING
        public boolean heliumEncoders = true;

        public double TRACKWIDTH = 30; // 30.0 for MKi
        public double WHEELBASE = 30; // 30.0 for MKi
        public double GEAR_RATIO = 8.14;
        public double WHEEL_RADIUS = .1143;

        // DRIVER SETTINGS
        public static int driver = 0;
        public static double transFactor = 1.0;
        public static double rotFactor = .30;

        // AUTON CONSTANTS
        public double XControllerP = 1.6878;
        public double XControllerD = 0;
        public double ThetaControllerP = 0;
        public double ThetaControllerD = 0;
        public RobotConfig autoConfig;

        // BASE CHASSIS CONFIGURATION
        public static final double MAX_VELOCITY = 5.4;
        public static final double MAX_ANGULAR_VELOCITY = Math.PI/6;
        public static final String[] LAYOUT_TITLE = { "Front Left", "Front Right", "Back Left", "Back Right" };
        public static final int[] CHASSIS_ID = { 2, 3, 4, 5 }; // FL, FR, BL, BR
        public static final int[] ENCODER_ID = { 7, 8, 9, 10 }; // FL, FR, BL, BR
        public static double[] ENCODER_OFFSETS = {-0.87890625, -0.996337890625, -0.638427734375, -0.892822265625};
        public static final int PIGEON_ID = 6;

        public Constants(){
            try {
                autoConfig = RobotConfig.fromGUISettings();
            } catch (IOException | org.json.simple.parser.ParseException e) {
                e.printStackTrace();
            }
            SwitchDriver(driver);
        }

        public static void SwitchDriver(int driver){
            switch (driver) {                
                default:
                transFactor = 1.0;
                rotFactor = .5;
                    break;
            }
        }
    }

    public interface Encoder {
        public void zero();
        public boolean connected();
        public double angle();
        public AngularVelocity velocity();

    }

    public static class Cancoder implements Encoder {

        CANcoder encoder;
        MagnetSensorConfigs magnetConfig = new MagnetSensorConfigs();

        public Cancoder(int id) {
            encoder = new CANcoder(id);
            magnetConfig.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
            magnetConfig.AbsoluteSensorDiscontinuityPoint = 1.0;
            magnetConfig.withMagnetOffset(Swerve.Constants.ENCODER_OFFSETS[id-7]);
            encoder.getConfigurator().apply(magnetConfig);
        }

        public void zero() {
            magnetConfig.MagnetOffset = -encoder.getAbsolutePosition().getValueAsDouble();
            encoder.getConfigurator().apply(magnetConfig);
        }

        public boolean connected(){
            return encoder.isConnected();
        }

        public double angle() {
            return ((encoder.getAbsolutePosition().getValue().in(Radians) + PI2) % PI2);
        }

        public AngularVelocity velocity(){
            return encoder.getVelocity().getValue();
        }
    }

    public static class Canand implements Encoder {
        Canandmag encoder;

        public Canand(int id) {
            encoder = new Canandmag(id);
            CanandmagSettings settings = new CanandmagSettings();
            settings.setInvertDirection(true);
            encoder.clearStickyFaults();
            encoder.setSettings(settings);
        }

        public void zero() {
            encoder.setAbsPosition(0, 250);
        }

        public boolean connected(){
            return encoder.isConnected();
        }

        public double angle() {
            return (encoder.getAbsPosition() * PI2) % PI2;
        }

        public AngularVelocity velocity(){
            return RadiansPerSecond.of(encoder.getVelocity());
        }
    }
}