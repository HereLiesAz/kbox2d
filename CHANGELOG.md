# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/en/1.0.0/)
and this project adheres to [Semantic Versioning](http://semver.org/spec/v2.0.0.html).

## [v3.2.0] - Unreleased

### Changed
- Converted the core library in `kbox2d-library` to idiomatic Kotlin. This includes the `common` package with classes like `Vec2`, `Vec3`, `Mat22`, `Mat33`, `Transform`, `Rot`, `Sweep`, `MathUtils`, and `Settings`.
- Updated the package names from `de.pirckheimer_gymnasium.jbox2d.common` to `com.hereliesaz.jbox2d.common`.

### Added
- Comprehensive KDoc documentation for the public API of the core library.
- A new "Usage" section with a public API guide in the `README.md`.

### Removed
- Obsolete Java source files and directories from the `kbox2d-library` module.

## [v3.1.0](https://github.com/engine-pi/jbox2d/releases/tag/v3.1.0) - 2024-08-05

<small>[Compare with v3.0.0](https://github.com/engine-pi/jbox2d/compare/v3.0.0...v3.1.0)</small>

### Added

35ffd6f676c55b0126ac01a3b0351a161abbf025) by Josef Friedrich).
- Add animated gifs ([daf7ff2](https://github.com/engine-pi/jbox2d/commit/daf7ff2ee13a101109616875a3f5b1d6040f8536) by Josef Friedrich).
- Add some permalinks ([55c5fc1](https://github.com/engine-pi/jbox2d/commit/55c5fc17c2fa352e494dae26e5c3505d643948f2) by Josef Friedrich).

### Removed

- Remove some hungarian m_ prefixes ([1506f20](https://github.com/engine-pi/jbox2d/commit/1506f200d70463628a3c0634d395a7fee1783728) by Josef Friedrich).

## [v3.0.0](https://github.com/engine-pi/jbox2d/releases/tag/v3.0.0) - 2024-08-02

<small>[Compare with v2.3.1](https://github.com/engine-pi/jbox2d/compare/v2.3.1...v3.0.0)</small>

### Changed

- Remove the `m_` (member?) `e_` (element?) `k_` (konstant?) attribute prefixes hungarian notation ([ddd3705](ddd3705893772e2cbad370c601e8c3dcf66dd577),
  [b1f4084](b1f408448543bc0a2232695cf5e3439814e6b18b),
  [5792d64](5792d640ed8f32843709894a919917fe47fe3e70),
  [89fee5c](89fee5c75e48577efe20153eda19a37b280cb42f) and
  [d0a6926](d0a6926adc41a4cc0743cbea1f14ca24b37037f9) by Josef Friedrich).

## [v2.3.1](https://github.com/engine-pi/jbox2d/releases/tag/v2.3.1) - 2024-08-01

<small>[Compare with jbox2d-2.3.0-BETA](https://github.com/engine-pi/jbox2d/compare/jbox2d-2.3.0-BETA...v2.3.1)</small>

### Added

- Add github action to deploy to maven central ([2537285](https://github.com/engine-pi/jbox2d/commit/253728551e60a8bc9759933f7aeddbfec9668827) by Josef Friedrich).

### Fixed

- Fix javadoc ([1b3893d](1b3893dbab0c3fa005c10c09df1b4129854e4367) by Josef Friedrich).
- Update dependencies ([7fa775d](7fa775d624f68ab498f1853201c4dcee4ace6376) by Josef Friedrich).

### Changed

- Reformat code style using engine pi style ([59514bb](59514bbfbbee67bfa80255bb4289f647ad77dc85) by Josef Friedrich).
- Move packages from org.jbox2d into de.pirckheimer_gymnasium.jbox2d ([a5c909d](a5c909de2b24a70fec1ef2bcb949aa5da36feb6c) by Josef Friedrich).

forked from https://github.com/jbox2d/jbox2d/commit/94bb3e4a706a6d1a5d8728a722bf0af9924dde84
