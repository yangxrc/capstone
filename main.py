from utils import GeneticOptimizer
from utils import PathPlanner
import numpy as np
import matplotlib.pyplot as plt
def optimize_drone_paths(sensor_positions: np.ndarray, n_drones: int = 4) -> dict:
    """Main function to optimize drone paths"""
    optimizer = GeneticOptimizer(sensor_positions, n_drones)
    best_parameters = optimizer.optimize()
    return best_parameters

def plot_drone_paths(drone_paths, sensor_positions=None, save_path=None):
    """
    plot 3-d figure
    
    Args:
        drone_paths: list all drones path point
        sensor_positions: sensor position
        save_path: figure path
    """
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    #set diffrent color for each drone
    colors = ['b', 'r', 'g', 'c', 'm', 'y', 'k', 'orange']
    
    # plot each drone's path
    for i, path in enumerate(drone_paths):
        path = np.array(path)
        color = colors[i % len(colors)]
        
        # plot each path
        ax.plot(path[:, 0], path[:, 1], path[:, 2], 
                color=color, label=f'Drone {i+1}', alpha=0.6)
        
        # mark start point and end point
        ax.scatter(path[0, 0], path[0, 1], path[0, 2], 
                  color=color, marker='o', s=100, label=f'Start {i+1}')
        ax.scatter(path[-1, 0], path[-1, 1], path[-1, 2], 
                  color=color, marker='s', s=100, label=f'End {i+1}')
    
    # plot sensor position if provided
    if sensor_positions is not None:
        ax.scatter(sensor_positions[:, 0], 
                  sensor_positions[:, 1], 
                  sensor_positions[:, 2], 
                  color='red', marker='^', s=100, label='Sensors')

    # set figure attribute
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('Drone Flight Paths')
    
    # add legend
    ax.legend()
    
    # set view
    ax.view_init(elev=20, azim=45)
    
    # save pictirue
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
    
    plt.show()

if __name__ == "__main__":
    sensor_positions = np.array([
    [10, 10, 0],
    [20, 20, 0],
    [30, 30, 0],
    [12,13,0],
    [24,35,0],

])


    best_parameters = optimize_drone_paths(sensor_positions, n_drones=3)


    planner = PathPlanner(sensor_positions, n_drones=3)
    time, scanned, crashes = planner.simulate(best_parameters)


    drone_paths = [agent.path for agent in planner.agents]  
      

    plot_drone_paths(drone_paths, sensor_positions)