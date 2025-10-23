import numpy as np

class ApexController:
    """
    Raibert-style height loop for the 1-D vertical task.
    Targets apex height h_star by adding a tiny stance energy pulse.
    """
    def __init__(self, k_h=40.0, alpha=1.0):
        self.k_h = float(k_h)     # proportional gain on apex error
        self.alpha = float(alpha) # maps u_h to rest-length tweak

    def predict_next_apex(self, z, zdot, params):
        # cheap predictor: ballistic energy at liftoff -> apex height
        m, g = params["m"], params["g"]
        return z + (zdot**2)/(2*g)

    def stance_rest_length(self, z, zdot, params, h_star):
        """Return effective rest length l0' = l0 + alpha * u_h (tiny tweak)."""
        h_hat = self.predict_next_apex(z, zdot, params)
        u_h = self.k_h * (h_star - h_hat)               # proportional energy nudge
        return params["l0"] + self.alpha * u_h * 1e-4   # small physical tweak
