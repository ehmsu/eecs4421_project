from enum import Enum, auto

class State(Enum):
    IDLE = auto()
    STANCE = auto()
    FLIGHT = auto()
    FAULT = auto()

def touchdown(z, zdot, l0):   # crossing into compression, contact on
    return (z <= l0) and (zdot < 0.0)

def liftoff(z, zdot, l0, k, b, m, g):  # spring back to l0 & unloading
    in_contact = (z <= l0 + 1e-6)
    contact_force = max(0.0, k*(l0 - z)) - b*max(0.0, -zdot)  # simple proxy
    return (not in_contact) or (contact_force <= 0.0 and zdot >= 0.0)
