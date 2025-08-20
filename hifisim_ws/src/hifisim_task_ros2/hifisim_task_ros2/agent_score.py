import numpy as np
import matplotlib.pyplot as plt
import math

class AgentScore:
    def __init__(self, expect_time, multiplier, scale_type="linear"):
        """
        Parameters:
            expect_time (float): Expected completion time (T)
            multiplier (float): Multiplier such that at t = nT, score = 50
            scale_type (str): "linear" or "log" to choose the scoring formula.
        """
        self.expect_time = expect_time
        self.multiplier = multiplier
        self.scale_type = scale_type.lower()
        
    def get_speed_score(self, t):
        T = self.expect_time
        n = self.multiplier
        
        if self.scale_type == "linear":
            # For t <= T, score is 100.
            if t <= T:
                return 100.0
            # For t > T, score declines linearly:
            score = 100.0 - (50.0/(T*(n-1)))*(t - T)
            return max(score, 0.0)
        
        elif self.scale_type == "log":
            # For t <= T, score is 100.
            if t <= T:
                return 100.0
            # For t > T, use logarithmic decay:
            score = 100.0 - (50.0/math.log(n)) * math.log(t/T)
            return max(score, 0.0)
        
        else:
            raise ValueError("Unsupported scale_type. Use 'linear' or 'log'.")

def main():
    # Settings: expected time (T) and multiplier (n)
    T = 60         # expected completion time
    n = 2          # multiplier such that at t = nT, score = 50

    # Create scoring objects for both scales
    linear_scoring = AgentScore(expect_time=T, multiplier=n, scale_type="linear")
    log_scoring    = AgentScore(expect_time=T, multiplier=n, scale_type="log")
    
    # Determine the maximum time for plotting.
    # For linear scale, the score reaches 0 at t = T(2n-1)
    t_max_linear = T * (2*n - 1)
    # For log scale, the score reaches 0 at t = n^2 * T.
    t_max_log = T * (n**2)
    t_max = max(t_max_linear, t_max_log)
    
    # Create a range of time values from 0 to t_max.
    t_values = np.linspace(0, t_max, 500)
    
    # Compute the scores for each scale
    linear_scores = np.array([linear_scoring.get_speed_score(t) for t in t_values])
    log_scores    = np.array([log_scoring.get_speed_score(t) for t in t_values])

    linear_scoring = AgentScore(expect_time=T, multiplier=n, scale_type="linear")
    log_scoring    = AgentScore(expect_time=T, multiplier=n, scale_type="log")
    t = 10.0
    score = linear_scoring.get_speed_score(t) 
    print(f"Linear scale score at t = {t}: {score:.2f}", flush=True)
    score = log_scoring.get_speed_score(t)
    print(f"Log scale score at t = {t}: {score:.2f}", flush=True)
    t = 15.0
    score = linear_scoring.get_speed_score(t) 
    print(f"Linear scale score at t = {t}: {score:.2f}", flush=True)
    score = log_scoring.get_speed_score(t)
    print(f"Log scale score at t = {t}: {score:.2f}", flush=True)
    t = 20.0
    score = linear_scoring.get_speed_score(t) 
    print(f"Linear scale score at t = {t}: {score:.2f}", flush=True)
    score = log_scoring.get_speed_score(t)
    print(f"Log scale score at t = {t}: {score:.2f}", flush=True)
    t = 30.0
    score = linear_scoring.get_speed_score(t) 
    print(f"Linear scale score at t = {t}: {score:.2f}", flush=True)
    score = log_scoring.get_speed_score(t)
    print(f"Log scale score at t = {t}: {score:.2f}", flush=True)
    
    # Plot the two scoring functions
    plt.figure(figsize=(10, 6))
    plt.plot(t_values, linear_scores, label="Linear Scale", lw=2)
    plt.plot(t_values, log_scores, label="Logarithmic Scale", lw=2, linestyle='--')
    plt.xlabel("Completion Time (t)")
    plt.ylabel("Score")
    plt.title("Agent Speed Score")
    plt.legend()
    plt.grid(True)
    plt.show()


if __name__ == '__main__':
    main()
