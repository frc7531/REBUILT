// Copyright (c) 2026 FRC 10219
// https://github.com/Team10219
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import frc.robot.Constants;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;

@SuppressWarnings("unused")
public class LoggedTunableEnum<T extends Enum<T>> implements Supplier<T> {
  private static final String tableKey = "/Tuning";

  private final String key;
  private final Class<T> enumClass;

  private boolean hasDefault = false;
  private T defaultValue;
  private LoggedNetworkString dashboardString;

  private final Map<Integer, T> lastHasChangedValues = new HashMap<>();

  public LoggedTunableEnum(String dashboardKey, Class<T> enumClass) {
    this.key = tableKey + "/" + dashboardKey;
    this.enumClass = enumClass;
  }

  public LoggedTunableEnum(String dashboardKey, Class<T> enumClass, T defaultValue) {
    this(dashboardKey, enumClass);
    initDefault(defaultValue);
  }

  public void initDefault(T defaultValue) {
    if (!hasDefault) {
      hasDefault = true;
      this.defaultValue = defaultValue;

      if (Constants.tuningMode && !Constants.disableHAL) {
        dashboardString = new LoggedNetworkString(key, defaultValue.name());
      }
    }
  }

  /** Returns the current enum (from NT in tuning mode, otherwise default). */
  @Override
  public T get() {
    if (!hasDefault) {
      return null;
    }

    if (!(Constants.tuningMode && !Constants.disableHAL)) {
      return defaultValue;
    }

    String raw = dashboardString.get();
    if (raw == null) {
      return defaultValue;
    }

    try {
      return Enum.valueOf(enumClass, raw);
    } catch (Exception e) {
      // invalid string in NT -> fall back to default
      return defaultValue;
    }
  }

  /** Maps the current enum to any type (ex: ratio -> ratio.driveRatio). */
  public <R> R getMapped(Function<T, R> mapper) {
    return mapper.apply(get());
  }

  /** Convenience for mapping the current enum to a double. */
  public double getDouble(ToDoubleFunction<T> mapper) {
    return mapper.applyAsDouble(get());
  }

  /**
   * Minimal ToDoubleFunction to avoid importing java.util.function.ToDoubleFunction if you want.
   */
  @FunctionalInterface
  public interface ToDoubleFunction<E> {
    double applyAsDouble(E value);
  }

  public boolean hasChanged(int id) {
    T currentValue = get();
    T lastValue = lastHasChangedValues.get(id);

    if (lastValue == null || currentValue != lastValue) {
      lastHasChangedValues.put(id, currentValue);
      return true;
    }
    return false;
  }

  @SafeVarargs
  public static <E extends Enum<E>> void ifChanged(
      int id, Consumer<E[]> action, LoggedTunableEnum<E>... tunableEnums) {

    if (Arrays.stream(tunableEnums).anyMatch(t -> t.hasChanged(id))) {
      @SuppressWarnings("unchecked")
      E[] values = (E[]) new Enum[tunableEnums.length];
      for (int i = 0; i < tunableEnums.length; i++) {
        values[i] = tunableEnums[i].get();
      }
      action.accept(values);
    }
  }

  @SafeVarargs
  public static <E extends Enum<E>> void ifChanged(
      int id, Runnable action, LoggedTunableEnum<E>... tunableEnums) {
    ifChanged(id, values -> action.run(), tunableEnums);
  }
}
