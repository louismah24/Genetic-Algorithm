import genome 
from xml.dom.minidom import getDOMImplementation
from enum import Enum
import numpy as np
import random

# code written with no assistance start
class MotorType(Enum):
    PULSE = 1
    SINE = 2
    TRIANGLE = 3
    RANDOM = 4

# code written with no assitance end
class Motor:
    def __init__(self, control_waveform, control_amp, control_freq):
        # FOR TESTING: Force specific motor type
        # Uncomment ONE of these lines to test specific motors:
        

        # code written with no assistance start
        # self.motor_type = MotorType.TRIANGLE  # Test triangle only
        # self.motor_type = MotorType.RANDOM    # Test random only
        
        # Normal motor selection (comment out when testing)
        if control_waveform <= 0.25:
            self.motor_type = MotorType.PULSE
        elif control_waveform <= 0.5:
            self.motor_type = MotorType.SINE
        elif control_waveform <= 0.75:
            self.motor_type = MotorType.TRIANGLE
        else:
            self.motor_type = MotorType.RANDOM

       

        self.amp = control_amp
        self.freq = control_freq
        self.phase = 0
        
        # For random motor - controls how often it changes
        self.random_change_rate = max(0.01, control_freq)  # Use freq to control randomness rate
        self.random_counter = 0
        self.current_random_output = 0
    

    def get_output(self):
        self.phase = (self.phase + self.freq) % (np.pi * 2)
        
        if self.motor_type == MotorType.PULSE:
            if self.phase < np.pi:
                output = 1
            else:
                output = -1
            
        elif self.motor_type == MotorType.SINE:
            output = np.sin(self.phase)
            
        elif self.motor_type == MotorType.TRIANGLE:
            # Triangle wave: linear ramp up, then linear ramp down
            if self.phase < np.pi:
                # Rising edge: -1 to +1
                output = (2 * self.phase / np.pi) - 1
            else:
                # Falling edge: +1 to -1
                output = 3 - (2 * self.phase / np.pi)
                
        elif self.motor_type == MotorType.RANDOM:
            # Update random output based on frequency
            self.random_counter += 1
            if self.random_counter >= int(1.0 / self.random_change_rate):
                self.current_random_output = random.uniform(-1, 1)
                self.random_counter = 0
            output = self.current_random_output
        
        return output * self.amp if self.amp > 0 else output
         # code written with no assistance end
class Creature:
    def __init__(self, gene_count):
        self.spec = genome.Genome.get_gene_spec()
        self.dna = genome.Genome.get_random_genome(len(self.spec), gene_count)
        self.flat_links = None
        self.exp_links = None
        self.motors = None
        self.start_position = None
        self.last_position = None

    def get_flat_links(self):
        if self.flat_links == None:
            gdicts = genome.Genome.get_genome_dicts(self.dna, self.spec)
            self.flat_links = genome.Genome.genome_to_links(gdicts)
        return self.flat_links
    
    def get_expanded_links(self):
        self.get_flat_links()
        if self.exp_links is not None:
            return self.exp_links
        
        exp_links = [self.flat_links[0]]
        genome.Genome.expandLinks(self.flat_links[0], 
                                self.flat_links[0].name, 
                                self.flat_links, 
                                exp_links)
        self.exp_links = exp_links
        return self.exp_links

    def to_xml(self):
        self.get_expanded_links()
        domimpl = getDOMImplementation()
        adom = domimpl.createDocument(None, "start", None)
        robot_tag = adom.createElement("robot")
        for link in self.exp_links:
            robot_tag.appendChild(link.to_link_element(adom))
        first = True
        for link in self.exp_links:
            if first:# skip the root node! 
                first = False
                continue
            robot_tag.appendChild(link.to_joint_element(adom))
        robot_tag.setAttribute("name", "pepe") #  choose a name!
        return '<?xml version="1.0"?>' + robot_tag.toprettyxml()

    def get_motors(self):
        self.get_expanded_links()
        if self.motors == None:
            motors = []
            for i in range(1, len(self.exp_links)):
                l = self.exp_links[i]
                m = Motor(l.control_waveform, l.control_amp,  l.control_freq)
                motors.append(m)
            self.motors = motors 
        return self.motors 
    
    def update_position(self, pos):
        if self.start_position == None:
            self.start_position = pos
        else:
            self.last_position = pos

    def get_distance_travelled(self):
        if self.start_position is None or self.last_position is None:
            return 0
        p1 = np.asarray(self.start_position)
        p2 = np.asarray(self.last_position)
        dist = np.linalg.norm(p1-p2)
        return dist 

    def update_dna(self, dna):
        self.dna = dna
        self.flat_links = None
        self.exp_links = None
        self.motors = None
        self.start_position = None
        self.last_position = None
        
    # code written with no assistance start
    def get_motor_type_distribution(self):
        """Utility function to see what motor types this creature has"""
        motors = self.get_motors()
        distribution = {
            'PULSE': 0,
            'SINE': 0, 
            'TRIANGLE': 0,
            'RANDOM': 0
        }
        for motor in motors:
            distribution[motor.motor_type.name] += 1
        return distribution
    # code written with no assistance end