class PIDController:
    def __init__(self, kp, ki, kd, sample_time, max_command):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.T = sample_time
        self.max_command = max_command
        self.name = "PID_incomplete"

        self.reset()
        self.calculate_factors()
    
    def calculate_factors(self):
        self.b0 = self.kp + (self.ki * self.T / 2) + (2 * self.kd / self.T)
        self.b1 = self.ki * self.T - (4 * self.kd / self.T)
        self.b2 = -self.kp + (self.ki * self.T / 2) + (2 * self.kd / self.T)

    def reset(self):
        self.ep = 0.0
        self.epp = 0.0
        self.up = 0.0
        self.upp = 0.0

    def control(self, yr, y):
        error = yr - y
        u = self.upp + self.b0*error + self.b1*self.ep + self.b2*self.epp
        u = min(max(u, -self.max_command), self.max_command)

        self.epp = self.ep
        self.ep = error

        self.upp = self.up
        self.up = u

        return u


class PIDFilter:
    def __init__(self, kp, ki, kd, sample_time):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.T = sample_time
        self.name = "PID_Filter"
        
        self.reset()
        self.calculate_factors()

    def calculate_factors(self):
        u0 = 4*self.kd + 2*self.kp*self.T + self.T*self.T*self.ki
        self.u1 = (2*self.T*self.T*self.ki - 8*self.kd)/u0
        self.u2 = (4*self.kd - 2*self.T*self.kp + self.T*self.T*self.ki)/u0
        self.xc = self.ki*self.T*self.T / u0

    def reset(self):
        self.xp = 0.0
        self.xpp = 0.0
        self.up = 0.0
        self.upp = 0.0

    def control(self, xr):
        u = self.xc*(xr + 2*self.xp + self.xpp) - self.u1*self.up - self.u2*self.upp
        self.upp = self.up
        self.up = u
        self.xpp = self.xp
        self.xp = xr

        return u


class FullPIDController:
    def __init__(self, kp, ki, kd, sample_time, max_command):
        self.PID = PIDController(kp, ki, kd, sample_time, max_command)
        self.filter = PIDFilter(kp, ki, kd, sample_time)
        self.name = "PID"
    
    def calculate_factors(self):
        self.PID.calculate_factors()
        self.filter.calculate_factors()

    def reset(self):
        self.PID.reset()
        self.filter.reset()

    def control(self, yr, y):
        yr_f = self.filter.control(yr)
        return self.PID.control(yr_f, y)


class PVController:
    def __init__(self, kp, kv, max_steering_command):
        self.kp = kp
        self.kv = kv
        self.max_steer = max_steering_command
        self.name = "PV"

    def reset(self):
        pass

    def control(self, xr, x, theta):
        u = self.kv * (self.kp*(xr - x) - theta)
        return min(max(u, -self.max_steer), self.max_steer)


class PFController:
    def __init__(self, kx, kff, max_command):
        self.kx = kx
        self.kff = kff
        self.max_command = max_command
        self.name = "PF"

    def reset(self):
        pass
    
    def control(self, yr, y):
        u = self.kx * (yr - y) + self.kff * yr
        return min(max(u, -self.max_command), self.max_command)
