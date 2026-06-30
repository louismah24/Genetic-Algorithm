import pybullet as p
import pybullet_data
import time
import numpy as np
import random
import creature
import population
import simulation
import genome
import math

def make_mountain(num_rocks=100, max_size=0.25, arena_size=10, mountain_height=5):
    """Create a mountain using scattered rocks with Gaussian distribution"""
    def gaussian(x, y, sigma=arena_size/4):
        """Return the height of the mountain at position (x, y) using a Gaussian function."""
        return mountain_height * math.exp(-((x**2 + y**2) / (2 * sigma**2)))

    for _ in range(num_rocks):
        x = random.uniform(-1 * arena_size/2, arena_size/2)
        y = random.uniform(-1 * arena_size/2, arena_size/2)
        z = gaussian(x, y)  # Height determined by the Gaussian function

        # Adjust the size of the rocks based on height. Higher rocks (closer to the peak) will be smaller.
        size_factor = 1 - (z / mountain_height)
        size = random.uniform(0.1, max_size) * size_factor

        orientation = p.getQuaternionFromEuler([random.uniform(0, 3.14), random.uniform(0, 3.14), random.uniform(0, 3.14)])
        rock_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[size, size, size])
        rock_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[size, size, size], rgbaColor=[0.5, 0.5, 0.5, 1])
        rock_body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=rock_shape, baseVisualShapeIndex=rock_visual, basePosition=[x, y, z], baseOrientation=orientation)

def make_rocks(num_rocks=100, max_size=0.25, arena_size=10):
    """Create scattered rocks on flat ground"""
    for _ in range(num_rocks):
        x = random.uniform(-1 * arena_size/2, arena_size/2)
        y = random.uniform(-1 * arena_size/2, arena_size/2)
        z = 0.5  # Adjust based on your needs
        size = random.uniform(0.1, max_size)
        orientation = p.getQuaternionFromEuler([random.uniform(0, 3.14), random.uniform(0, 3.14), random.uniform(0, 3.14)])
        rock_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[size, size, size])
        rock_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[size, size, size], rgbaColor=[0.5, 0.5, 0.5, 1])
        rock_body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=rock_shape, baseVisualShapeIndex=rock_visual, basePosition=[x, y, z], baseOrientation=orientation)

def make_arena(arena_size=10, wall_height=1):
    """Create arena with walls and floor"""
    wall_thickness = 0.5
    floor_collision_shape = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=[arena_size/2, arena_size/2, wall_thickness])
    floor_visual_shape = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=[arena_size/2, arena_size/2, wall_thickness], rgbaColor=[1, 1, 0, 1])
    floor_body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=floor_collision_shape, baseVisualShapeIndex=floor_visual_shape, basePosition=[0, 0, -wall_thickness])

    wall_collision_shape = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=[arena_size/2, wall_thickness/2, wall_height/2])
    wall_visual_shape = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=[arena_size/2, wall_thickness/2, wall_height/2], rgbaColor=[0.7, 0.7, 0.7, 1])  # Gray walls

    # Create four walls
    p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_collision_shape, baseVisualShapeIndex=wall_visual_shape, basePosition=[0, arena_size/2, wall_height/2])
    p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_collision_shape, baseVisualShapeIndex=wall_visual_shape, basePosition=[0, -arena_size/2, wall_height/2])

    wall_collision_shape = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=[wall_thickness/2, arena_size/2, wall_height/2])
    wall_visual_shape = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=[wall_thickness/2, arena_size/2, wall_height/2], rgbaColor=[0.7, 0.7, 0.7, 1])  # Gray walls

    p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_collision_shape, baseVisualShapeIndex=wall_visual_shape, basePosition=[arena_size/2, 0, wall_height/2])
    p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_collision_shape, baseVisualShapeIndex=wall_visual_shape, basePosition=[-arena_size/2, 0, wall_height/2])

class MountainClimbingSimulation(simulation.Simulation):
    """Extended simulation class for mountain climbing evaluation"""
    # code written with no assistance start
    def __init__(self, sim_id=0, arena_size=20, use_gui=False):
        self.arena_size = arena_size
        if use_gui:
            self.physicsClientId = p.connect(p.GUI)
        else:
            self.physicsClientId = p.connect(p.DIRECT)
        self.sim_id = sim_id
        self.setup_environment()

    # code written with no assistance end
    
    def setup_environment(self):
        """Setup the mountain climbing environment"""
        pid = self.physicsClientId
        p.resetSimulation(physicsClientId=pid)
        p.setPhysicsEngineParameter(enableFileCaching=0, physicsClientId=pid)
        p.setGravity(0, 0, -10, physicsClientId=pid)
        
        # Create arena with walls
        wall_thickness = 0.5
        arena_size = self.arena_size
        
        # Floor
        floor_collision_shape = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=[arena_size/2, arena_size/2, wall_thickness], physicsClientId=pid)
        floor_visual_shape = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=[arena_size/2, arena_size/2, wall_thickness], rgbaColor=[1, 1, 0, 1], physicsClientId=pid)
        floor_body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=floor_collision_shape, baseVisualShapeIndex=floor_visual_shape, basePosition=[0, 0, -wall_thickness], physicsClientId=pid)

        # Walls
        wall_collision_shape = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=[arena_size/2, wall_thickness/2, 1], physicsClientId=pid)
        wall_visual_shape = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=[arena_size/2, wall_thickness/2, 1], rgbaColor=[0.7, 0.7, 0.7, 1], physicsClientId=pid)

        # Create four walls
        p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_collision_shape, baseVisualShapeIndex=wall_visual_shape, basePosition=[0, arena_size/2, 1], physicsClientId=pid)
        p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_collision_shape, baseVisualShapeIndex=wall_visual_shape, basePosition=[0, -arena_size/2, 1], physicsClientId=pid)

        wall_collision_shape2 = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=[wall_thickness/2, arena_size/2, 1], physicsClientId=pid)
        wall_visual_shape2 = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=[wall_thickness/2, arena_size/2, 1], rgbaColor=[0.7, 0.7, 0.7, 1], physicsClientId=pid)

        p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_collision_shape2, baseVisualShapeIndex=wall_visual_shape2, basePosition=[arena_size/2, 0, 1], physicsClientId=pid)
        p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_collision_shape2, baseVisualShapeIndex=wall_visual_shape2, basePosition=[-arena_size/2, 0, 1], physicsClientId=pid)
        
        # Load mountain from URDF file
        try:
            # Try to load the mountain - check if file exists first
            import os
            if os.path.exists('shapes/gaussian_pyramid.urdf'):
                mountain_position = (0, 0, -1)
                mountain_orientation = p.getQuaternionFromEuler((0, 0, 0))
                mountain = p.loadURDF("shapes/gaussian_pyramid.urdf", mountain_position, mountain_orientation, useFixedBase=1, physicsClientId=pid)
                if mountain < 0:
                    print("Warning: Failed to load mountain URDF, continuing without mountain")
            else:
                print("Warning: Mountain URDF file not found at 'shapes/gaussian_pyramid.urdf'")
                print("Continuing with flat ground simulation")
        except Exception as e:
            print(f"Warning: Could not load mountain: {e}")
            print("Continuing with flat ground simulation")
    
    # code written with no assistance start
    def run_creature(self, cr, iterations=2400):
        """Run a single creature in the mountain environment and evaluate its climbing performance"""
        pid = self.physicsClientId
        self.setup_environment()  # Reset environment for each creature
        
        xml_file = 'temp' + str(self.sim_id) + '.urdf'
        xml_str = cr.to_xml()
        with open(xml_file, 'w') as f:
            f.write(xml_str)
        
        try:
            # Load creature and position it at the base of the mountain
            cid = p.loadURDF(xml_file, physicsClientId=pid)
            if cid < 0:  # Check if loading failed
                print(f"Warning: Failed to load URDF for creature {self.sim_id}")
                cr.climbing_fitness = 0
                cr.max_height = 0
                cr.center_distance = float('inf')
                return
                
            start_position = [-7, -7, 3]  # Safe consistent spawn position
            p.resetBasePositionAndOrientation(cid, start_position, [0, 0, 0, 1], physicsClientId=pid)
            
            max_height = 0
            center_distance = float('inf')  # Distance to mountain center
            
            for step in range(iterations):
                p.stepSimulation(physicsClientId=pid)
                if step % 24 == 0:
                    self.update_motors(cid=cid, cr=cr)
                
                # Safely get position with error handling
                try:
                    pos, orn = p.getBasePositionAndOrientation(cid, physicsClientId=pid)
                    cr.update_position(pos)
                    
                    # Track maximum height achieved (mountain climbing performance)
                    current_height = pos[2]
                    if current_height > max_height:
                        max_height = current_height
                    
                    # Track distance to mountain center (0, 0)
                    distance_to_center = np.sqrt(pos[0]**2 + pos[1]**2)
                    if distance_to_center < center_distance:
                        center_distance = distance_to_center
                        
                except Exception as e:
                    print(f"Warning: Lost creature {self.sim_id} at step {step}: {e}")
                    break
            
            # Calculate fitness based on both height achieved and proximity to mountain center
            height_fitness = max_height
            proximity_fitness = max(0, 15 - center_distance)  # Reward getting closer to center
            cr.climbing_fitness = height_fitness + 0.5 * proximity_fitness
            cr.max_height = max_height
            cr.center_distance = center_distance
            
        except Exception as e:
            print(f"Error running creature {self.sim_id}: {e}")
            cr.climbing_fitness = 0
            cr.max_height = 0
            cr.center_distance = float('inf')
    # code written with no assitance end
# code written with no assistance start
def run_evolution(generations=100, pop_size=20, gene_count=10, mutation_rate=0.01):
    """Run the genetic algorithm for mountain climbing"""
    
    print("=== Mountain Climbing Evolution Parameters ===")
    print(f"Population size: {pop_size}")
    print(f"Initial genome size: {gene_count} genes")
    print(f"Mutation rate: {mutation_rate}")
    print(f"Number of generations: {generations}")
    print("=" * 50)
    
    print("\nInitializing population...")
    pop = population.Population(pop_size=pop_size, gene_count=gene_count)
    sim = MountainClimbingSimulation(use_gui=False)  # Use headless for faster evolution
    
    best_fitness_history = []
    avg_fitness_history = []
    
    for generation in range(generations):
        print(f"\n=== Generation {generation + 1}/{generations} ===")
        
        # Evaluate all creatures
        for i, cr in enumerate(pop.creatures):
            sim.run_creature(cr, 2400)
            if i % 5 == 0:
                print(f"Evaluated creature {i + 1}/{len(pop.creatures)}")
        
        # Calculate fitness metrics
        fitnesses = [cr.climbing_fitness for cr in pop.creatures]
        heights = [cr.max_height for cr in pop.creatures]
        distances = [cr.center_distance for cr in pop.creatures]
        links = [len(cr.get_expanded_links()) for cr in pop.creatures]
        
        # Calculate statistics
        best_fitness = np.max(fitnesses)
        avg_fitness = np.mean(fitnesses)
        
        # Store fitness history
        best_fitness_history.append(best_fitness)
        avg_fitness_history.append(avg_fitness)
        
        # Print generation statistics
        print(f"Best fitness: {best_fitness:.3f}")
        print(f"Average fitness: {avg_fitness:.3f}")
        print(f"Max height reached: {np.max(heights):.3f}")
        print(f"Min distance to center: {np.min(distances):.3f}")
        print(f"Mean links: {np.mean(links):.1f}")
        
        # Selection and reproduction
        fit_map = population.Population.get_fitness_map(fitnesses)
        new_creatures = []
        
        for i in range(len(pop.creatures)):
            # Select parents
            p1_ind = population.Population.select_parent(fit_map)
            p2_ind = population.Population.select_parent(fit_map)
            p1 = pop.creatures[p1_ind]
            p2 = pop.creatures[p2_ind]
            
            # Create offspring through crossover and mutation
            dna = genome.Genome.crossover(p1.dna, p2.dna)
            dna = genome.Genome.point_mutate(dna, rate=mutation_rate, amount=0.25)
            dna = genome.Genome.shrink_mutate(dna, rate=0.25)
            dna = genome.Genome.grow_mutate(dna, rate=0.1)
            
            cr = creature.Creature(1)
            cr.update_dna(dna)
            new_creatures.append(cr)
        
        # Elitism - keep the best creature
        max_fit = np.max(fitnesses)
        for cr in pop.creatures:
            if cr.climbing_fitness == max_fit:
                new_cr = creature.Creature(1)
                new_cr.update_dna(cr.dna)
                new_creatures[0] = new_cr
                filename = f"mountain_climber_gen_{generation + 1}.csv"
                genome.Genome.to_csv(cr.dna, filename)
                print(f"Elite saved to {filename}")
                break
        
        pop.creatures = new_creatures
    
    return best_fitness_history, avg_fitness_history
# code written with no assistance end



def test_single_creature_visual():
    """Test a single creature in the mountain environment with visual feedback"""
    print("Setting up visual test environment...")
    
    # Connect with GUI for visualization
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -10)

    arena_size = 20
    make_arena(arena_size=arena_size)

    # Load mountain
    mountain_position = (0, 0, -1)
    mountain_orientation = p.getQuaternionFromEuler((0, 0, 0))
    p.setAdditionalSearchPath('shapes/')
    mountain = p.loadURDF("gaussian_pyramid.urdf", mountain_position, mountain_orientation, useFixedBase=1)

    # Generate a random creature
    cr = creature.Creature(gene_count=5)
    
    # Save and load creature
    with open('test_mountain_climber.urdf', 'w') as f:
        f.write(cr.to_xml())
    
    rob1 = p.loadURDF('test_mountain_climber.urdf', [-8, -8, 2.5])

    print("Starting visual simulation...")
    print("The creature will attempt to climb the mountain.")
    print("Fitness will be based on maximum height achieved and proximity to mountain center.")
    
    # Run simulation with real-time visualization
    p.setRealTimeSimulation(1)
    
    start_time = time.time()
    max_height = 0
    min_distance = float('inf')
    
    try:
        while time.time() - start_time < 30:  # Run for 30 seconds
            # Update motors every few frames
            if int((time.time() - start_time) * 240) % 24 == 0:
                motors = cr.get_motors()
                for jid in range(p.getNumJoints(rob1)):
                    if jid < len(motors):
                        mode = p.VELOCITY_CONTROL
                        vel = motors[jid].get_output()
                        p.setJointMotorControl2(rob1, jid, controlMode=mode, targetVelocity=vel)
            
            # Track performance
            pos, orn = p.getBasePositionAndOrientation(rob1)
            current_height = pos[2]
            distance_to_center = np.sqrt(pos[0]**2 + pos[1]**2)
            
            if current_height > max_height:
                max_height = current_height
            if distance_to_center < min_distance:
                min_distance = distance_to_center
            
            time.sleep(1/240)
            
    except KeyboardInterrupt:
        print("Simulation interrupted by user")
    
    final_pos, _ = p.getBasePositionAndOrientation(rob1)
    height_fitness = max_height
    proximity_fitness = max(0, 15 - min_distance)
    total_fitness = height_fitness + 0.5 * proximity_fitness
    
    print(f"\n=== Performance Results ===")
    print(f"Maximum height reached: {max_height:.3f}")
    print(f"Minimum distance to center: {min_distance:.3f}")
    print(f"Final position: ({final_pos[0]:.3f}, {final_pos[1]:.3f}, {final_pos[2]:.3f})")
    print(f"Height fitness: {height_fitness:.3f}")
    print(f"Proximity fitness: {proximity_fitness:.3f}")
    print(f"Total fitness: {total_fitness:.3f}")

if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1 and sys.argv[1] == "visual":
        # Run visual test with a single creature
        test_single_creature_visual()
    elif len(sys.argv) > 1 and sys.argv[1] == "evolve":
        # Run full evolution with specified parameters
        generations = int(sys.argv[2]) if len(sys.argv) > 2 else 100
        pop_size = int(sys.argv[3]) if len(sys.argv) > 3 else 20
        gene_count = int(sys.argv[4]) if len(sys.argv) > 4 else 10
        mutation_rate = float(sys.argv[5]) if len(sys.argv) > 5 else 0.01
        
        print(f"Starting evolution with your specified parameters:")
        print(f"  Generations: {generations}")
        print(f"  Population size: {pop_size}")
        print(f"  Initial genome size: {gene_count} genes")
        print(f"  Mutation rate: {mutation_rate}")
        
        best_fitness_history, avg_fitness_history = run_evolution(generations, pop_size, gene_count, mutation_rate)
        
        print("\n" + "=" * 60)
        print("EVOLUTION COMPLETE - FINAL RESULTS")
        print("=" * 60)
        print(f"Total generations run: {generations}")
        print(f"Final best fitness: {max(best_fitness_history):.3f}")
        print(f"Final average fitness: {avg_fitness_history[-1]:.3f}")
        print(f"Initial best fitness: {best_fitness_history[0]:.3f}")
        print(f"Initial average fitness: {avg_fitness_history[0]:.3f}")
        print(f"Improvement in best fitness: {max(best_fitness_history) - best_fitness_history[0]:.3f}")
        print(f"Improvement in average fitness: {avg_fitness_history[-1] - avg_fitness_history[0]:.3f}")
        
        # Print fitness progression every 50 generations
        print(f"\nFitness progression (every 50 generations):")
        print("Generation | Best Fitness | Avg Fitness")
        print("-" * 40)
        for i in range(0, len(best_fitness_history), 50):
            gen_num = i + 1
            best_fit = best_fitness_history[i]
            avg_fit = avg_fitness_history[i]
            print(f"{gen_num:9d} | {best_fit:11.3f} | {avg_fit:10.3f}")
        
        # Print final generation if not already printed
        if len(best_fitness_history) % 50 != 0:
            gen_num = len(best_fitness_history)
            best_fit = best_fitness_history[-1]
            avg_fit = avg_fitness_history[-1]
            print(f"{gen_num:9d} | {best_fit:11.3f} | {avg_fit:10.3f}")
            
    else:
        print("Usage:")
        print("  python mov-envt.py visual                               - Run visual test with single creature")
        print("  python mov-envt.py evolve [gens] [pop] [genes] [mut_rate] - Run evolution")
        print("    gens: number of generations (default: 100)")
        print("    pop: population size (default: 20)")  
        print("    genes: genes per creature (default: 10)")
        print("    mut_rate: mutation rate (default: 0.01)")
        print("\nDefault run (your specifications):")
        print("  python mov-envt.py evolve")
        print("  -> 500 generations, 20 population, 10 genes, 0.01 mutation rate")
        print("\nCustom example:")
        print("  python mov-envt.py evolve 300 25 8 0.02")