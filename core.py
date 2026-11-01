"""
Simplified aircraft takeoff and climbout simulation.

Model assumptions:
- Alpha is treated as pitch angle.
- CL = 0.5 * alpha + 0.25, with alpha in degrees.
- Lift acts vertically.
- Drag acts horizontally.
- Aerodynamic forces use horizontal velocity squared.
- Constant thrust, constant density, level runway, no wind.
"""

import math

import pandas as pd
import matplotlib.pyplot as plt


# Aircraft inputs
wing_area = 16.17          # m²
wing_span = 11.0           # m
aircraft_mass = 1157.0     # kg
rolling_friction_coef = 0.02
thrust = 2000.0            # N

# Environment
rho = 1.225                # kg/m³
g = 9.81                  # m/s²

# Aerodynamic inputs
drag_coef_else = 0.03
oswald_e = 0.8
rotation_alpha = 7.0       # degrees; treated as pitch
rotation_speed = 30.9      # m/s

# Simulation settings
time_step = 0.01           # seconds
steps = 30000


class Airplane:
    def __init__(
        self,
        wing_Area=wing_area,
        wing_Span=wing_span,
        rolling_Friction=rolling_friction_coef,
        mass=aircraft_mass,
        Thrust=thrust,
    ):
        self.wing_area = wing_Area
        self.wing_span = wing_Span
        self.aspect_ratio = self.wing_span**2 / self.wing_area
        self.rolling_friction = rolling_Friction
        self.mass = mass
        self.Thrust = Thrust

        self.reset()

    def reset(self):
        # Horizontal state: x, xdot, xdouble_dot
        self.position = 0.0
        self.velocity = 0.0
        self.acceleration = 0.0

        # Vertical state: y, ydot, ydouble_dot
        self.altitude = 0.0
        self.vertical_velocity = 0.0
        self.vertical_acceleration = 0.0

        self.alpha = 0.0
        self.liftcoef = 0.25
        self.drag_coef = drag_coef_else

        self.lift = 0.0
        self.drag = 0.0
        self.normal_force = self.mass * g
        self.rolling_res = 0.0

        self.rotated = False
        self.rotation_time = None

        self.time_history = []
        self.position_history = []
        self.velocity_history = []
        self.acceleration_history = []
        self.altitude_history = []
        self.vertical_velocity_history = []
        self.vertical_acceleration_history = []
        self.stage_history = []

    def calculate_forces(self):
        # Retain your specified lift-curve relationship.
        self.liftcoef = 0.5 * self.alpha + 0.25

        self.drag_coef = drag_coef_else + (
            self.liftcoef**2
            / (math.pi * self.aspect_ratio * oswald_e)
        )

        # Your simplified model uses xdot² for both lift and drag.
        q = 0.5 * rho * self.velocity**2

        self.lift = q * self.wing_area * self.liftcoef
        self.drag = q * self.wing_area * self.drag_coef

    def climbout(self, dt):
        """Advance one post-rotation timestep in x and y."""
        self.alpha = rotation_alpha
        self.calculate_forces()

        alpha_rad = math.radians(self.alpha)
        weight = self.mass * g

        thrust_x = self.Thrust * math.cos(alpha_rad)
        thrust_y = self.Thrust * math.sin(alpha_rad)

        # Vertical force before accounting for runway support.
        vertical_force = thrust_y + self.lift - weight

        on_ground = (
            self.altitude <= 0.0
            and self.vertical_velocity <= 0.0
        )

        # Runway supports the aircraft until upward forces
        # are sufficient for liftoff.
        if on_ground:
            self.normal_force = max(0.0, -vertical_force)
        else:
            self.normal_force = 0.0

        self.rolling_res = (
            self.rolling_friction * self.normal_force
        )

        # xdouble_dot = (T*cos(alpha) - Crr*N - D) / m
        self.acceleration = (
            thrust_x - self.rolling_res - self.drag
        ) / self.mass

        # ydouble_dot = (T*sin(alpha) + L - W + N) / m
        self.vertical_acceleration = (
            vertical_force + self.normal_force
        ) / self.mass

        # Integrate horizontal and vertical velocities.
        # This model is restricted to forward motion.
        self.velocity = max(
            0.0,
            self.velocity + self.acceleration * dt,
        )
        self.vertical_velocity += (
            self.vertical_acceleration * dt
        )

        # Integrate horizontal distance and altitude.
        self.position += self.velocity * dt
        self.altitude += self.vertical_velocity * dt

        # Simple runway boundary, not a landing-impact model.
        if self.altitude < 0.0:
            self.altitude = 0.0
            self.vertical_velocity = 0.0

    def Takeoff(self):
        self.reset()
        self.record_state(0.0, "pre_rotation")

        for step in range(steps):
            start_time = step * time_step

            # Rotation is a one-way stage transition.
            if (
                not self.rotated
                and self.velocity >= rotation_speed
            ):
                self.rotated = True
                self.rotation_time = start_time

            if self.rotated:
                # Post-rotation motion is handled separately.
                self.climbout(time_step)
                stage = "post_rotation"

            else:
                # PRE-ROTATION: horizontal ground roll.
                self.alpha = 0.0
                self.calculate_forces()

                weight = self.mass * g

                self.normal_force = max(
                    0.0,
                    weight - self.lift,
                )
                self.rolling_res = (
                    self.rolling_friction * self.normal_force
                )

                # m*vdot = T - Crr*N - 0.5*rho*v²*Cd*A
                # Since v = xdot, vdot = xdouble_dot.
                self.acceleration = (
                    self.Thrust - self.rolling_res - self.drag
                ) / self.mass

                # Ground-roll stage constrains y motion.
                self.vertical_acceleration = 0.0
                self.vertical_velocity = 0.0
                self.altitude = 0.0

                self.velocity = max(
                    0.0,
                    self.velocity + self.acceleration * time_step,
                )
                self.position += self.velocity * time_step
                stage = "pre_rotation"

            self.record_state(
                (step + 1) * time_step,
                stage,
            )

    def record_state(self, elapsed_time, stage):
        self.time_history.append(elapsed_time)
        self.position_history.append(self.position)
        self.velocity_history.append(self.velocity)
        self.acceleration_history.append(self.acceleration)
        self.altitude_history.append(self.altitude)
        self.vertical_velocity_history.append(
            self.vertical_velocity
        )
        self.vertical_acceleration_history.append(
            self.vertical_acceleration
        )
        self.stage_history.append(stage)

    def Plot(self):
        df = pd.DataFrame({
            "Time": self.time_history,
            "Position": self.position_history,
            "Velocity": self.velocity_history,
            "Acceleration": self.acceleration_history,
            "Altitude": self.altitude_history,
            "VerticalVelocity": self.vertical_velocity_history,
            "VerticalAcceleration": (
                self.vertical_acceleration_history
            ),
            "Stage": self.stage_history,
        })

        df.to_csv("flight_simulation_data.csv", index=False)

        fig, axes = plt.subplots(
            4,
            1,
            figsize=(11, 12),
            sharex=True,
            constrained_layout=True,
        )

        plot_settings = [
            ("Position", "Horizontal distance (m)", "tab:blue"),
            ("Velocity", "Horizontal velocity (m/s)", "tab:orange"),
            (
                "Acceleration",
                "Horizontal acceleration (m/s²)",
                "tab:red",
            ),
            ("Altitude", "Altitude above runway (m)", "tab:green"),
        ]

        for ax, (column, ylabel, color) in zip(
            axes, plot_settings
        ):
            ax.plot(
                df["Time"],
                df[column],
                color=color,
                label=column,
            )

            if self.rotation_time is not None:
                ax.axvline(
                    self.rotation_time,
                    color="gray",
                    linestyle="--",
                    label="Rotation begins",
                )

            ax.set_ylabel(ylabel)
            ax.grid(True, alpha=0.3)
            ax.legend(loc="best")

        axes[-1].set_xlabel("Time (s)")
        fig.suptitle("Simplified takeoff and climbout simulation")
        plt.show()


if __name__ == "__main__":
    airplane = Airplane()
    airplane.Takeoff()
    airplane.Plot()