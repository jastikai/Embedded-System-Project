import sys
import os
import time
import threading
import tkinter as tk
from tkinter import ttk
from tkinter.scrolledtext import ScrolledText

# Add the 'src' directory to sys.path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), 'src')))
from paho.mqtt import client as mqtt

# Constants
BROKER_ADDRESS = "192.168.1.153"
TOPIC_COMMAND = "esp32/game/command"
TOPIC_RESPONSE = "esp32/game/response"
DEPLETION_INTERVAL = 5  # Time in seconds between each depletion

# Game state
game_state = {
    "need_to_play": 10,
    "need_to_talk": 10,
    "need_to_feed": 10
}

game_running = True  # Flag to control the game loop
awaiting_response = False  # Flag to indicate if waiting for a response
last_action = None  # To track the last action sent to the ESP32

# Initialize MQTT client
client = mqtt.Client()

# Function to create a custom progress bar
def create_bar(canvas, x, y, width, height, initial_value):
    canvas.create_rectangle(x, y, x + width, y + height, outline="black", fill="grey")
    bar = canvas.create_rectangle(x, y, x + (width * initial_value / 10), y + height, outline="black", fill="green")
    return bar

# Function to update the custom progress bars
def update_bars():
    update_bar(play_canvas, play_bar, game_state['need_to_play'])
    update_bar(talk_canvas, talk_bar, game_state['need_to_talk'])
    update_bar(feed_canvas, feed_bar, game_state['need_to_feed'])

# Function to update a single custom progress bar
def update_bar(canvas, bar, value):
    color = "green" if value > 3 else "red"
    canvas.itemconfig(bar, fill=color)
    canvas.coords(bar, 10, 10, 10 + (200 * value / 10), 30)

# Function to log messages in the GUI
def log_message(message):
    log_box.config(state=tk.NORMAL)
    log_box.insert(tk.END, message + "\n")
    log_box.config(state=tk.DISABLED)
    log_box.yview(tk.END)  # Scroll to the end

# MQTT Callbacks
def on_connect(client, userdata, flags, rc):
    log_message("Connected to MQTT broker with result code " + str(rc))
    client.subscribe(TOPIC_RESPONSE)

def on_message(client, userdata, msg):
    global last_action, awaiting_response
    message = msg.payload.decode()
    log_message(f"Message received: {message}")
    if message == "ACTION OK" and last_action:
        log_message(f"ESP32 action {last_action} was successful!")
        game_state[last_action] = 10  # Refill the "bar" to 10
        update_bars()
    elif message == "ACTION FAILED":
        log_message(f"ESP32 action {last_action} failed!")

    # Reset flags after receiving a response
    last_action = None
    awaiting_response = False

# Function to deplete the game state
def deplete_game_state():
    global game_running, awaiting_response  # Declare awaiting_response as global
    while game_running:
        for need, value in game_state.items():
            if game_state[need] > 0:
                game_state[need] -= 1

        update_bars()

        # Check if any need drops to 0 (pet "dies")
        for need, value in game_state.items():
            if value == 0:
                log_message(f"Your pet's {need.split('_')[-1]} reached 0. The pet has died.")
                client.publish(TOPIC_COMMAND, "stop_game")  # Send stop game command to ESP32
                game_running = False  # Stop the game
                log_message("Game over. Thanks for playing!")
                client.loop_stop()
                window.quit()  # Close the GUI
                return  # Exit the deplete_game_state loop

        # Check if any need drops to 3
        for need, value in game_state.items():
            if value == 3 and not awaiting_response:
                need_name = need.split('_')[-1]  # Extract the action name (play, talk, feed)
                log_message(f"Pet needs {need_name}! Sending alert to ESP32.")
                client.publish(TOPIC_COMMAND, f"alert_{need_name}")
        
        time.sleep(DEPLETION_INTERVAL)

# Action button command
def send_action(action):
    global awaiting_response, last_action
    if awaiting_response:
        log_message("Waiting for response from ESP32...")
        return

    key = f"need_to_{action}"
    client.publish(TOPIC_COMMAND, action)
    last_action = key  # Set the last action to wait for the response
    awaiting_response = True  # Block further input until response is received
    log_message(f"Sent '{action}' command to ESP32.")

# GUI setup
window = tk.Tk()
window.title("Pet Game")

# Create canvases for custom progress bars
ttk.Label(window, text="Play Level").pack()
play_canvas = tk.Canvas(window, width=220, height=40)
play_canvas.pack()
play_bar = create_bar(play_canvas, 10, 10, 200, 20, game_state['need_to_play'])

ttk.Label(window, text="Talk Level").pack()
talk_canvas = tk.Canvas(window, width=220, height=40)
talk_canvas.pack()
talk_bar = create_bar(talk_canvas, 10, 10, 200, 20, game_state['need_to_talk'])

ttk.Label(window, text="Feed Level").pack()
feed_canvas = tk.Canvas(window, width=220, height=40)
feed_canvas.pack()
feed_bar = create_bar(feed_canvas, 10, 10, 200, 20, game_state['need_to_feed'])

# Action buttons
ttk.Button(window, text="Play", command=lambda: send_action("play")).pack(pady=5)
ttk.Button(window, text="Talk", command=lambda: send_action("talk")).pack(pady=5)
ttk.Button(window, text="Feed", command=lambda: send_action("feed")).pack(pady=5)

# Log box
log_box = ScrolledText(window, height=10, state=tk.DISABLED)
log_box.pack(fill=tk.BOTH, expand=True)

# MQTT setup
client.on_connect = on_connect
client.on_message = on_message
client.connect(BROKER_ADDRESS, 1883, 60)
client.loop_start()

# Start the depletion process in a separate thread
depletion_thread = threading.Thread(target=deplete_game_state)
depletion_thread.daemon = True  # Ensure the thread exits when the main program does
depletion_thread.start()

# Start the GUI event loop
window.mainloop()

# Ensure the game exits cleanly
client.loop_stop()
