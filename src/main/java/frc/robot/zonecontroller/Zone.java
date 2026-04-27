// Copyright (c) 2026 FRC 10219
// https://github.com/Team10219
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.zonecontroller;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.Supplier;

// Zone system inspired by FRC Team 4481 (Team Rembrandts).
// Their 2026 Build thread describes using a Zone interface that returns WPILib
// Triggers directly, allowing zone based command bindings to live cleanly in
// RobotContainer alongside all other trigger bindings
// https://www.chiefdelphi.com/t/frc-4481-team-rembrandts-2026-build-thread-open-alliance/507296

public interface Zone {

  boolean containsPoint(Translation2d point);

  default Trigger contains(Supplier<Translation2d> translationSupplier) {
    return new Trigger(() -> containsPoint(translationSupplier.get()));
  }

  /* Returns a zone that is active when inside this zone OR the other zone. */
  default Zone union(Zone other) {
    return CompositeZone.union(this, other);
  }

  /* Returns a zone that is active when inside this zone AND the other zone. */
  default Zone intersection(Zone other) {
    return CompositeZone.intersection(this, other);
  }

  /* Returns a zone that is active when inside this zone but NOT the other zone. */
  default Zone difference(Zone other) {
    return CompositeZone.difference(this, other);
  }

  /* Returns a zone that is active when NOT inside this zone. */
  default Zone complement() {
    return CompositeZone.complement(this);
  }
}
