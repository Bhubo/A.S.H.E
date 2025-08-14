ASHE — Autonomous Sword for Heroic Engagements
Vodou-Enabled Companion Blade (Firmware Rev. L)

ASHE is a high-performance, interactive companion blade designed to be a peripheral within the Vodou Ecosystem. It is a stateless, responsive device that comes to life when bound to a master controller, such as the Vodou Binding Glove. Its core purpose is to act as a physical manifestation of the wielder's will, instantly reflecting their personalized settings and reacting to their movements with light, sound, and haptic feedback.

Core Philosophy: The Stateless Vessel
The most important design principle of ASHE is that it is a stateless vessel. It holds no memory of its own.

No Onboard Memory: ASHE does not save any user settings to its internal flash memory. Every time it powers on, it boots into a neutral, default state.

Imprinted by the Wielder: The sword is designed to be "bound" by a master Vodou controller. This binding process imprints the user's personal Equilibrium (their unique profile of colors, sounds, and settings) onto the sword for the duration of the session.

Seamless Transition: This architecture ensures that any user can pick up any ASHE unit, and with their personal Vodou Glove, instantly make it their sword. When the session ends, the sword reverts to its neutral state, ready for the next wielder.

This design makes the hardware interchangeable and places the user's identity at the center of the ecosystem, where it belongs.

Key Features
Non-Blocking Effects Engine: The firmware is built on a millis()-based state machine with zero delay() calls. This ensures the sword is always responsive to motion and BLE commands, even while playing complex light and sound effects.

Real-time Motion Detection: Utilizes a BNO085 IMU to detect high-speed swings and sharp clashes, triggering unique audio-visual effects for each.

Dynamic Haptic Feedback: An integrated vibration motor provides haptic feedback, including a soft-start feature and a high-power Overdrive Mode.

Procedural Audio: All sound effects (swing, clash, boot, flourish) are generated procedurally, eliminating the need for an SD card and preserving hardware simplicity.

Seamless Binding Ritual: A single, powerful BIND command from a master controller imprints the user's settings and initiates a confirmation ritual, signifying the link between wielder and blade.

FPU-Optimized: The code leverages the nRF52840's hardware Floating-Point Unit (FPU) for smooth, efficient calculations for lighting effects and sensor fusion.

The Vodou Protocol (BLE UART)
ASHE is controlled via a simple, text-based command protocol over a BLE UART service.

Binding & Equilibrium
The binding ritual is the core of the Vodou experience. It's a single, seamless command that establishes the link and syncs the user's profile.

Command

Example

Description

BIND <payload>

BIND A=255,128,0|B=255,60,0

The primary command. Instantly applies the user's Equilibrium settings from the payload and then begins the Binding Ritual effect to confirm the link.

Core State Commands
These commands control the primary state of the sword.

Command

Description

ARM

Arms the sword, enabling motion detection and effects.

DISARM

Disarms the sword, turning off all effects and motors.

OVERDRIVE ON

Engages Overdrive Mode: full motor power and an unstable light flicker.

OVERDRIVE OFF

Disengages Overdrive Mode.

PING

A simple connection check. ASHE will reply with PONG.

Manual & Standalone Control
For use without a master glove (e.g., with the PAPA UI phone app), ASHE can be controlled with these granular commands.

Command

Example

Description

<target> <color>

A purple or B #FF8000

Sets the color of a specific LED group (A, B, or BOTH).

LIGHTS OFF

-

Turns all LEDs off.

MOTOR ON

-

Engages the motor with a soft start (disabled in Overdrive).

MOTOR OFF

-

Disengages the motor (disabled in Overdrive).

MOTOR PWM <val>

MOTOR PWM 150

Sets the motor speed to a specific value from 0-255 (disabled in Overdrive).

Hardware Requirements
MCU: Seeed Studio XIAO nRF52840

IMU: SparkFun BNO080/BNO085 9-DOF Sensor

Lighting: Common-anode RGB LEDs (2 groups)

Haptics: Vibration Motor with a MOSFET driver

Audio: Piezo buzzer or a small amplifier + speaker

Power: 3.7V LiPo Battery
