# This section of the code is used to collect data for training the dynamic model.
# It works in conjunction with global_vision_for_inversedyna, where the global vision code provides state feedback for the robotic fish.
# The dataset format is: (red_marker_x, red_marker_y, control action of the robotic fish)

# Host Computer User Guide:
# amplitude input box: Input the amplitude for continuous motion or the maximum value during single-word collection.
# frequency/factor input box: Input the frequency of continuous motion or the asymmetry of the motion curve during single-word collection.
# bias/steps input box: Input the bias during continuous motion or the number of action steps during a single collection.

# Required Operations:
# First, click "set parameter" to initialize all parameters and serial port settings, and create an instance of the action generator in the code's lower layers.
# Click "connect" to establish the serial connection.
# Click "init fish" to complete the handshake and motor initialization with the robotic fish.

# During operation, the motor's parameters are updated in real-time through "set parameter" during continuous motion.
# During operation, clicking "disconnect" will end the current collection, and the data will be saved.

# start collection: The robotic fish moves continuously, collecting real-time states and actions. Click again to stop collection and save.
# single collection: Collects the robotic fish's state and action once. The motion of the robotic fish is determined by the input parameters in the text boxes, and the process ends automatically after execution and is saved.
# random collection: Randomly generates an action for the robotic fish, executes it, and saves the motion state and action.
