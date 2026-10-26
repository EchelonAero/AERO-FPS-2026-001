'''
Runs complete physics engine for flight simulation. 

V0.0.1 - Thrust applied to physics model
'''


#Imports
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


#Variables
wing_area = 120
aircraft_mass = 15
rolling_friction_coef = 0.8
runway_elevation = 50
runway_slope = 3
temp = 20
wind = 3
flaps = 1
thrust = 100
time_step = 0.01
steps = 30000
step = 0


class Airplane:
    def __init__(self,wing_Area: float = 120, rolling_Friction: float = 0.8, Flaps: int = 1, mass: float = 15, Thrust: float = 100):
        self.wing_area = wing_Area
        self.rolling_friction = rolling_Friction
        self.Flaps = Flaps
        self.mass = mass
        self.Thrust = Thrust
        self.acceleration = 0.0
        self.velocity = 0.0
        self.position = 0.0


    
    def Takeoff(self):
        self.time_history = []
        self.position_history = []
        self.velocity_history = []
        self.acceleration_history = []

        for step in range(steps):
            elapsed_time = step * time_step

            # Use full thrust for the first 2 seconds, then 40%.
            thrust = self.Thrust if elapsed_time < 2 else self.Thrust * 0.4
            self.acceleration = thrust / self.mass

            # Advance the aircraft state by one time step.
            self.velocity += self.acceleration * time_step
            self.position += self.velocity * time_step

            self.time_history.append(elapsed_time)
            self.position_history.append(self.position)
            self.velocity_history.append(self.velocity)
            self.acceleration_history.append(self.acceleration)
                
    def Plot(self):
        data = {
            'Time': self.time_history,
            'Position': self.position_history,
            'Velocity': self.velocity_history,
            'Acceleration': self.acceleration_history
        }
        df = pd.DataFrame(data)
        df.to_csv('flight_simulation_data.csv', index=False)
        plt.figure(figsize=(10, 6))
        plt.plot(df['Time'], df['Acceleration'], label='Acceleration (m/s^2)')
        plt.xlabel('Time (s)')
        plt.ylabel('Acceleration (m/s^2)')
        plt.title('Flight Simulation Data')
        plt.legend()
        plt.show()

Airplane1 = Airplane(wing_Area=wing_area, rolling_Friction=rolling_friction_coef, Flaps=flaps, mass=aircraft_mass, Thrust=thrust)
Airplane1.Takeoff()
Airplane1.Plot()

        


