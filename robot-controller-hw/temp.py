params = {
    'kp': 2.2,            # Proportional gain
    'ki': 1.6,            # Integral gain
    'kd': 0.0,            # Derivative gain
    'kt': 1.0,            # Tracking gain
    'output_min': -10.0,  # Minimum output limit
    'output_max': 10.0,   # Maximum output limit
    'update_period': 0.1   # Update period in seconds
}

state = {
    'integral': 0.0,
    'prev_error': 0.0
}

def pid_controller(setpoint, measurement, state, params):
    error = setpoint - measurement
    state['integral'] += error * params['update_period']
    derivative = (error - state['prev_error']) / params['update_period']
    state['prev_error'] = error

    output = (params['kp'] * error +
                params['ki'] * state['integral'] +
                params['kd'] * derivative)

    
    # Tracking anti-windup
    output_clamped = output
    if output > params['output_max']:
        output_clamped = params['output_max']
    elif output < params['output_min']:
        output_clamped = params['output_min']

    integral_correction = (output - output_clamped) * params['update_period']
    state['integral'] -= integral_correction

    return output_clamped

import matplotlib.pyplot as plt

sp = 20
pv = 0

data = {
    "Process Variable" : [],
    "Control Signal" : [],
    "Integrator" : [],
}

for i in range(100):
    control_signal = pid_controller(sp, pv, state, params)
    pv += control_signal * 0.1  # Simulate system response
    data["Control Signal"].append(control_signal)
    data["Process Variable"].append(pv)
    data['Integrator'].append(state['integral'])

for k in data:
    plt.plot(data[k], label=k)
plt.legend()
plt.grid()
plt.show()