package com.michaelgrundvig.frc.world.physical;

import static com.michaelgrundvig.frc.world.config.Rules.notBlank;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;

/**
 * What something is made of, by name: carpet, polycarbonate, aluminum, a tread's rubber. Only a
 * name so far. How two materials grip and bounce off each other belongs to the pair, so those
 * numbers come with the simulator's contacts, each pair's from where it was measured.
 *
 * @param name what it's called, the same everywhere it's used: two things named alike are made of
 *     the same stuff
 */
@ConfigRecord
public record Material(String name) {
  /** The field's carpet: what raised ground is covered in unless it says otherwise. */
  public static final Material CARPET = builder().name("carpet").build();

  /** Aluminum: what a mechanism's links are drawn in unless they say otherwise. */
  public static final Material ALUMINUM = builder().name("aluminum").build();

  public Material {
    Checks.on(Material.class).field("name", name, notBlank()).orThrow();
  }

  /**
   * A material by its name: {@code Material.of("polycarbonate")}.
   *
   * @param name what it's called, the same everywhere it's used
   */
  public static Material of(String name) {
    return builder().name(name).build();
  }

  /** Starts describing a material, one named value at a time. */
  static MaterialBuilder.NameStage builder() {
    return MaterialBuilder.builder();
  }
}
