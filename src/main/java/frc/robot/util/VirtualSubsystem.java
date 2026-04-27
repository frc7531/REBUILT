// Copyright (c) 2026 FRC 10219
// https://github.com/Team10219
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

public abstract class VirtualSubsystem {
  private static List<VirtualSubsystem> subsystems = new ArrayList<>();

  public VirtualSubsystem() {
    subsystems.add(this);
  }

  public static void periodicAll() {
    for (VirtualSubsystem subsystem : subsystems) {
      subsystem.periodic();
    }
  }

  public abstract void periodic();
}
