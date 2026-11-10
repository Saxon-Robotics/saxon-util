package org.saxonrobotics.saxonutil.lib.io.sensors.encoder;

import static org.saxonrobotics.saxonutil.dependencies.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import org.saxonrobotics.saxonutil.dependencies.PhoenixUtil;
import org.wpilib.units.measure.Angle;

public class EncoderIOCANcoder implements EncoderIO {
  private final CANcoder cancoder;

  private final StatusSignal<Angle> position;

  public EncoderIOCANcoder(
      CANBus canbus, int id, CANcoderConfiguration config, double frequencyHz) {
    cancoder = new CANcoder(id, canbus);
    tryUntilOk(5, () -> cancoder.getConfigurator().apply(config));
    position = cancoder.getAbsolutePosition();
    position.setUpdateFrequency(frequencyHz);
    PhoenixUtil.registerSignals(canbus, position);
  }

  public EncoderIOCANcoder(CANBus canbus, int id, CANcoderConfiguration config) {
    this(canbus, id, config, 100.0);
  }

  @Override
  public void updateInputs(EncoderIOInputs inputs) {
    inputs.connected = position.getStatus().isOK();
    inputs.absolutePosition = position.getValue();
  }

  public int getDeviceID() {
    return cancoder.getDeviceID();
  }
}
