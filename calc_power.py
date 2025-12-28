import pandas as pd

# ==========================================
# 1. THE PHYSICS ENGINE (The Formulas)
# ==========================================
class MotorModel:
    def __init__(self):
        self.IDLE_CURRENT = 0.18  # Amps (Base load)
        self.SLOPE = 2.52         # Torque-to-Current Slope
        
    def get_inrush_penalty(self, accel_setting):
        """
        Calculates D(y): The 'Inrush Penalty' based on acceleration.
        Values derived from empirical lab testing.
        """
        if accel_setting == 0:
            return 2.00  # Max Perf (Hard Start) - Massive Spike
        elif accel_setting <= 10:
            return 0.60  # Fast (Heavy Jerk)
        elif accel_setting <= 20:
            return 0.20  # Medium (Standard)
        else:
            return 0.15  # Soft (Optimized/Golden) - Minimal Spike

    def calculate_current(self, torque_pct, accel_setting, n_motors):
        """
        The Master Formula:
        I_total = N * [ (2.52 * x) + 0.18 + D(y) ]
        """
        # 1. Calculate Static Load (The Weight)
        # Formula: (2.52 * Torque%) + 0.18
        i_static = (self.SLOPE * torque_pct) + self.IDLE_CURRENT
        
        # 2. Calculate Dynamic Inrush (The Acceleration Tax)
        i_dynamic = self.get_inrush_penalty(accel_setting)
        
        # 3. Sum and Multiply by N motors
        total_current = n_motors * (i_static + i_dynamic)
        
        return round(total_current, 2)

# Physics engine
engine = MotorModel()

# ==========================================
# 2. SIMULATION RUNNERS
# ==========================================

def simulate_arms():
    print("\n" + "="*60)
    print(">>> SIMULATION 1: ARMS (Bus A+B)")
    print("Constraint: 10.0 Amps (Car Outlet) | Motors: 4")
    print("="*60)
    
    # Define the scenarios to test
    # [Name, Torque(x), Accel(y), Velocity, Motors]
    scenarios = [
        ("Max Perf.",     0.80,      0,        600,      4),
        ("Fast Lift",     0.30,      10,       400,      4),
        ("Heavy Jerk",    0.80,      10,       100,      4),
        ("Golden Sett.",  0.80,      40,       200,      4)
    ]
    
    data = []
    for name, t, a, v, n in scenarios:
        load = engine.calculate_current(t, a, n)
        status = "FAIL" if load > 10.0 else "PASS"
        data.append([name, f"{int(t*100)}%", a, v, f"{load} A", status])
        
    df = pd.DataFrame(data, columns=["Scenario", "Torque", "Accel", "Vel", "Total Load", "Result"])
    print(df.to_string(index=False))

def simulate_mobility():
    print("\n" + "="*60)
    print(">>> SIMULATION 2: MOBILITY (Bus C)")
    print("Constraint: 3.0 Amps (USB-C) | Motors: 3")
    print("="*60)
    
    # Note: We lowered Golden Drive Torque to 22% to pass the 3A limit. 
    # Still fails the neck though bcs it's physically impossible to 
    # run 5 motors on 3A RIP
    scenarios = [
        ("Stall/Push",    0.80,      20,       0,        3),
        ("Fast Dash",     0.50,      0,        800,      3),
        ("Golden Drive",  0.22,      20,       600,      3) 
    ]
    
    data = []
    for name, t, a, v, n in scenarios:
        load = engine.calculate_current(t, a, n)
        status = "FAIL" if load > 3.0 else "PASS"
        data.append([name, f"{int(t*100)}%", a, v, f"{load} A", status])

    df = pd.DataFrame(data, columns=["Scenario", "Torque", "Accel", "Vel", "Total Load", "Result"])
    print(df.to_string(index=False))

def simulate_neck():
    print("\n" + "="*60)
    print(">>> SIMULATION 3: NECK (Shared Bus C)")
    print("Constraint: Dynamic (3.0A - Wheel_Load)")
    print("="*60)
    
    # Neck scenarios rely on what the wheels are doing
    # Note: We use the "Golden Drive" settings (22% T, 20 Acc) for wheels here
    scenarios = [
        # Name,           Wheel_Trq, Wheel_Acc, Neck_Trq, Neck_Acc
        ("Wheels Idle",   0.00,      40,        0.40,     20), 
        ("Wheels Cruise", 0.22,      20,        0.40,     10), # Fail Scenario
        ("Wheels Cruise", 0.22,      20,        0.20,     40)  # Safe Mode
    ]
    
    data = []
    for name, w_t, w_a, n_t, n_a in scenarios:
        # 1. Calculate Wheel Load (Background)
        wheel_load = engine.calculate_current(w_t, w_a, 3)
        
        # 2. Calculate Neck Load (Foreground)
        neck_load = engine.calculate_current(n_t, n_a, 2)
        
        # 3. Sum Total Bus Load
        total = wheel_load + neck_load
        
        status = "FAIL" if total > 3.0 else "PASS"
        
        # Formatting for table
        w_state = f"{wheel_load} A"
        total_s = f"{total} A"
        
        data.append([name, f"{int(n_t*100)}%", n_a, w_state, total_s, status])
        
    df = pd.DataFrame(data, columns=["Scenario", "Neck Trq", "Neck Acc", "Wheel Load", "Total Bus", "Result"])
    print(df.to_string(index=False))

# ==========================================
# 3. EXECUTE
# ==========================================
if __name__ == "__main__":
    simulate_arms()
    simulate_mobility()
    simulate_neck()