class PIDController:
    def __init__(self, kp, ki, kd, sample_time, max_command):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.T = sample_time
        self.name = "PID"

        self.ep = 0
        self.epp = 0
        self.up = 0
        self.upp = 0

        self.b0 = self.kp + (self.ki * self.T / 2) + (2 * self.kd / self.T)
        self.b1 = self.ki * self.T - (4 * self.kd / self.T)
        self.b2 = -self.kp + (self.ki * self.T / 2) + (2 * self.kd / self.T)

    def control(self, yr, y):
        error = yr - y
        u = self.upp + self.b0*error + self.b1*self.ep + self.b2*self.epp
        u = min(max(u, -self.max_command), self.max_command)

        self.epp = self.ep
        self.ep = error

        self.upp = self.up
        self.up = u

        return u


class PVController:
    def __init__(self, kp, kv, sample_time, L, v, max_command):
        self.kp = kp
        self.kv = kv
        self.T = sample_time
        self.max_command = max_command
        self.name = "PV"
        
        self.L = L
        self.v = v

    def control(self, xr, x, theta):
        u = self.kv*self.L/self.v * (self.kp*(xr - x) - theta)
        return min(max(u, -self.max_command), self.max_command)


class PFController:
    def __init__(self, kx, kff, max_command):
        self.kx = kx
        self.kff = kff
        self.max_command = max_command
        self.name = "PF"

    def control(self, yr, y):
        u = self.kx * (yr - y) + self.kff * yr
        return min(max(u, -self.max_command), self.max_command)
