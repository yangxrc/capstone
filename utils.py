import numpy as np
from typing import List, Tuple
import random

class Agent:
    def __init__(self, position: np.ndarray, mass: float = 5.0, max_velocity: float = 10.0):
        self.position = position.astype(np.float64)
        self.velocity = np.zeros(3,dtype=np.float64)
        self.acceleration = np.zeros(3,dtype=np.float64)
        self.mass = mass
        self.max_velocity = max_velocity
        self.path = [position.copy()]
        
    def update(self, dt: float = 0.1):
        # Update velocity using acceleration
        self.velocity += self.acceleration * dt
        
        # Limit velocity to max_velocity
        velocity_magnitude = np.linalg.norm(self.velocity)
        if velocity_magnitude > self.max_velocity:
            self.velocity = self.velocity * self.max_velocity / velocity_magnitude
            
        # Update position using velocity
        self.position += self.velocity * dt
        self.path.append(self.position.copy())

class PathPlanner:
    def __init__(self, 
                 sensor_positions: np.ndarray,
                 n_drones: int = 4,
                 propulsion_force: float = 100.0,
                 transmission_radius: float = 2.0,
                 crash_radius: float = 2.0):
        
        self.sensor_positions = sensor_positions
        self.n_drones = n_drones
        self.propulsion_force = propulsion_force
        self.transmission_radius = transmission_radius
        self.crash_radius = crash_radius
        
        # Initialize drones at starting positions
        self.agents = []
        for i in range(n_drones):
            start_pos = np.array([-10, -10 + i*5, 0])  #set drone initial place
            self.agents.append(Agent(start_pos))
            
        self.unscanned_sensors = list(range(len(sensor_positions)))
        
    def calculate_distance(self, pos1: np.ndarray, pos2: np.ndarray) -> float:
        return np.linalg.norm(pos1 - pos2)
    
    def calculate_interaction_vector(self, 
                                  agent_pos: np.ndarray, 
                                  target_pos: np.ndarray,
                                  w_att: float,
                                  w_rep: float,
                                  c_att: float,
                                  c_rep: float) -> np.ndarray:
        """Calculate attraction-repulsion vector between agent and target"""
        distance = self.calculate_distance(agent_pos, target_pos)
        direction = (target_pos - agent_pos) / distance
        
        # Attraction-repulsion magnitude
        magnitude = w_att * np.exp(-c_att * distance) - w_rep * np.exp(-c_rep * distance)
        
        return magnitude * direction
    
    def get_propulsion_direction(self, 
                               agent_idx: int, 
                               parameters: dict) -> np.ndarray:
        """Calculate total propulsion direction for an agent"""
        agent = self.agents[agent_idx]
        
        # Calculate sensor interactions
        sensor_vector = np.zeros(3)
        for sensor_idx in self.unscanned_sensors:
            interaction = self.calculate_interaction_vector(
                agent.position,
                self.sensor_positions[sensor_idx],
                parameters['w_sensor_att'],
                parameters['w_sensor_rep'],
                parameters['c_sensor_att'],
                parameters['c_sensor_rep']
            )
            sensor_vector += interaction
            
        # Calculate other agent interactions
        agent_vector = np.zeros(3)
        for other_idx, other_agent in enumerate(self.agents):
            if other_idx != agent_idx:
                interaction = self.calculate_interaction_vector(
                    agent.position,
                    other_agent.position,
                    parameters['w_agent_att'],
                    parameters['w_agent_rep'],
                    parameters['c_agent_att'],
                    parameters['c_agent_rep']
                )
                agent_vector += interaction
                
        # Combine vectors with weights
        total_vector = (parameters['W_s'] * sensor_vector + 
                       parameters['W_a'] * agent_vector)
        
        # Normalize
        magnitude = np.linalg.norm(total_vector)
        if magnitude > 0:
            return total_vector / magnitude
        return total_vector
    
    def check_sensor_scanning(self):
        """Check if any sensors are within transmission range of drones"""
        sensors_to_remove = []
        for sensor_idx in self.unscanned_sensors:
            for agent in self.agents:
                distance = self.calculate_distance(
                    agent.position, 
                    self.sensor_positions[sensor_idx]
                )
                if distance <= self.transmission_radius:
                    sensors_to_remove.append(sensor_idx)
                    break
        
        for sensor_idx in sensors_to_remove:
            self.unscanned_sensors.remove(sensor_idx)
            
        return len(sensors_to_remove)
    
    def check_collisions(self) -> int:
        """Check for collisions between drones"""
        n_crashes = 0
        for i in range(len(self.agents)):
            for j in range(i+1, len(self.agents)):
                distance = self.calculate_distance(
                    self.agents[i].position,
                    self.agents[j].position
                )
                if distance <= self.crash_radius:
                    n_crashes += 1
        return n_crashes
    
    def simulate(self, parameters: dict, max_time: float = 1200.0) -> Tuple[float, int, int]:
        """Run simulation with given parameters"""
        dt = 0.1
        time = 0
        total_scanned = 0
        total_crashes = 0
        
        while time < max_time and self.unscanned_sensors:
            # Update agent positions
            for i, agent in enumerate(self.agents):
                # Calculate propulsion direction
                propulsion = self.get_propulsion_direction(i, parameters)
                
                # Update acceleration
                agent.acceleration = self.propulsion_force * propulsion / agent.mass
                
                # Update position and velocity
                agent.update(dt)
            
            # Check for scanned sensors and collisions
            total_scanned += self.check_sensor_scanning()
            total_crashes += self.check_collisions()
            
            time += dt
            
        return time, total_scanned, total_crashes

class GeneticOptimizer:
    def __init__(self, 
                 sensor_positions: np.ndarray,
                 n_drones: int = 4,
                 population_size: int = 7,
                 n_generations: int = 100):
        
        self.sensor_positions = sensor_positions
        self.n_drones = n_drones
        self.population_size = population_size
        self.n_generations = n_generations
        
        # Define parameter bounds
        self.param_bounds = {
            'W_s': (1, 10),
            'W_a': (1, 10),
            'w_sensor_att': (0, 1),
            'w_sensor_rep': (0, 1),
            'c_sensor_att': (0, 1),
            'c_sensor_rep': (0, 1),
            'w_agent_att': (0, 1),
            'w_agent_rep': (0, 1),
            'c_agent_att': (0, 1),
            'c_agent_rep': (0, 1)
        }
        
    def create_random_parameters(self) -> dict:
        """Create random parameter set within bounds"""
        return {
            key: random.uniform(bounds[0], bounds[1])
            for key, bounds in self.param_bounds.items()
        }
    
    def calculate_fitness(self, parameters: dict) -> float:
        """Calculate fitness score for parameter set"""
        planner = PathPlanner(self.sensor_positions, self.n_drones)
        time, scanned, crashes = planner.simulate(parameters)
        
        # Weights for different objectives
        w1, w2, w3 = 0.6, 0.1, 0.3
        
        # Calculate fitness components
        coverage_score = (len(self.sensor_positions) - scanned) / len(self.sensor_positions)
        time_score = time / 1200.0  # Normalized by max time
        crash_score = crashes / self.n_drones
        
        # Lower score is better
        return w1 * coverage_score + w2 * time_score + w3 * crash_score
    
    def crossover(self, parent1: dict, parent2: dict) -> dict:
        """Create offspring from two parents"""
        child = {}
        for key in self.param_bounds.keys():
            if random.random() < 0.5:
                child[key] = parent1[key]
            else:
                child[key] = parent2[key]
        return child
    
    def mutate(self, parameters: dict, mutation_rate: float = 0.1) -> dict:
        """Randomly mutate parameters"""
        mutated = parameters.copy()
        for key, bounds in self.param_bounds.items():
            if random.random() < mutation_rate:
                mutated[key] = random.uniform(bounds[0], bounds[1])
        return mutated
    
    def optimize(self) -> dict:
        """Run genetic algorithm optimization"""
        # Initialize population
        population = [self.create_random_parameters() 
                     for _ in range(self.population_size)]
        
        best_fitness = float(0.1)
        best_parameters = None
        
        for generation in range(self.n_generations):
            # Evaluate fitness
            fitness_scores = [(params, self.calculate_fitness(params)) 
                            for params in population]
            
            # Sort by fitness
            fitness_scores.sort(key=lambda x: x[1])
            
            # Update best solution
            if fitness_scores[0][1] < best_fitness:
                best_fitness = fitness_scores[0][1]
                best_parameters = fitness_scores[0][0].copy()
                
            # Select top performers
            top_performers = [params for params, _ in fitness_scores[:self.population_size//2]]
            
            # Create new population
            new_population = top_performers.copy()
            
            # Add offspring
            while len(new_population) < self.population_size:
                parent1 = random.choice(top_performers)
                parent2 = random.choice(top_performers)
                offspring = self.crossover(parent1, parent2)
                offspring = self.mutate(offspring)
                new_population.append(offspring)
                
            population = new_population
            
        return best_parameters


def optimize_drone_paths(sensor_positions: np.ndarray, n_drones: int = 4) -> dict:
    """Main function to optimize drone paths"""
    optimizer = GeneticOptimizer(sensor_positions, n_drones)
    best_parameters = optimizer.optimize()
    return best_parameters