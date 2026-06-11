"""GUI application for synchronizing and logging serial device input.

This module provides a Tkinter-based GUI for connecting to serial devices
(e.g., Arduino, Teensy), logging timestamped input events to CSV files,
and monitoring connection status.

Features:
    - Auto-detect available serial ports
    - Real-time connection status monitoring
    - CSV logging with timestamp and input columns
    - Connection timeout detection (5 second threshold)
    - Configurable output directory and filename

Functions:
    connect_to_arduino: Establish serial connection and start logging.
    log_data: Background thread function to read and log serial data.
    monitor_connection: Background thread to detect connection loss.
    update_gui: Update GUI display with status.
    update_clock: Refresh connection time display.
    browse_directory: Directory selection dialog.
    create_gui: Initialize Tkinter UI components.
    main: Application entry point.

Example:
    python -m tabletop_py.gaze.syncer

Dependencies:
    Requires pyserial, Pillow (PIL), and tkinter.
"""

import csv
import os
import sys
import threading
import time
from datetime import datetime
from tkinter import (
    Button,
    Entry,
    Frame,
    Label,
    StringVar,
    Tk,
    filedialog,
    messagebox,
)
from tkinter.ttk import Combobox

import serial
from PIL import Image, ImageTk
from serial.tools.list_ports import comports

#: Image display width in pixels
IMAGE_WIDTH = 250

#: Image display height in pixels
IMAGE_HEIGHT = 250


def connect_to_arduino(
    port,
    last_data_time,
    gui_labels,
    root,
    port_combobox,
    connect_button,
    filename,
    directory,
    filename_entry,
    directory_entry,
    browse_button,
):
    """
    Connect to the Arduino board and start logging data.

    Args:
        port (str): The serial port to connect to.
        last_data_time (list): A list containing the timestamp of the last received data.
        gui_labels (dict): A dictionary containing GUI labels and their associated data.
        root (Tk): The root window of the GUI.
        port_combobox (Combobox): The combobox for selecting the serial port.
        connect_button (Button): The connect button.
        filename (str): The filename for the CSV file.
        directory (str): The directory for saving the CSV file.
        filename_entry (Entry): The entry field for the filename.
        directory_entry (Entry): The entry field for the directory.
        browse_button (Button): The browse button for selecting the directory.
    """
    if not port:
        messagebox.showerror("Error", "Please select a port.")
        return
    if not filename:
        messagebox.showerror("Error", "Please enter a filename.")
        return
    if not filename.endswith(".csv"):
        messagebox.showerror("Error", "Filename must end with '.csv'.")
        return
    if not directory:
        messagebox.showerror("Error", "Please select a save directory.")
        return
    if not os.path.isdir(directory):
        messagebox.showerror("Error", "Invalid save directory.")
        return

    try:
        ser = serial.Serial(port, 115200, timeout=1)
        print(f"Connected on {port}.")
        last_data_time[0] = (
            time.time()
        )  # Reset last data time on successful connection
        log_thread = threading.Thread(
            target=log_data,
            args=(ser, last_data_time, gui_labels, filename, directory),
            daemon=True,
        )
        log_thread.start()
        port_combobox.config(state="disabled")  # Disable the combobox
        connect_button.config(state="disabled")  # Disable the connect button
        filename_entry.config(
            state="disabled"
        )  # Disable the filename entry field
        directory_entry.config(
            state="disabled"
        )  # Disable the directory entry field
        browse_button.config(state="disabled")  # Disable the browse button
        gui_labels["image_label"].config(
            image=gui_labels["image"]
        )  # Display image only after connection
        gui_labels["connect_label"].config(
            text=""
        )  # Remove the "Connect to the board" message
        start_monitoring(
            last_data_time, gui_labels
        )  # Start monitoring only after successful connection
        update_clock(gui_labels)  # Start updating the clock
    except serial.SerialException:
        gui_labels["connection_lost"] = True
        gui_labels["connect_label"].config(text="Connection failed")
        update_gui(
            gui_labels, False, "Connection failed"
        )  # Update GUI to show error status


def log_data(ser, last_data_time, gui_labels, filename, directory):
    """
    Log data received from the Arduino board to a CSV file.

    Args:
        ser (serial.Serial): The serial connection to the Arduino board.
        last_data_time (list): A list containing the timestamp of the last received data.
        gui_labels (dict): A dictionary containing GUI labels and their associated data.
        filename (str): The filename for the CSV file.
        directory (str): The directory for saving the CSV file.
    """
    file_path = os.path.join(directory, filename)
    with open(file_path, "w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["time", "input"])
        print(f"Logging started in {file_path}.")

    try:
        while True:
            if ser.in_waiting > 0:
                last_data_time[0] = time.time()
                line = ser.readline().decode().strip()
                if line:
                    timestamp, input = line.split(",")
                    with open(file_path, "a", newline="") as file:
                        writer = csv.writer(file)
                        writer.writerow([timestamp, input])
                        file.flush()
                    update_gui(gui_labels, True, "")
    except (serial.SerialException, OSError):
        lost_time = datetime.now().strftime("%I:%M:%S %p")
        gui_labels["connection_lost"] = True
        update_gui(gui_labels, False, f"Connection lost at {lost_time}")


def update_gui(gui_labels, has_data, message):
    """
    Update the GUI based on the connection status and received data.

    Args:
        gui_labels (dict): A dictionary containing GUI labels and their associated data.
        has_data (bool): Indicates whether data has been received from the Arduino board.
        message (str): The message to display on the GUI.
    """
    if not has_data:
        gui_labels["image_label"].config(image="")  # Remove image
        gui_labels["clock_label"].config(text=message)


def update_clock(gui_labels):
    """
    Update the clock display on the GUI.

    Args:
        gui_labels (dict): A dictionary containing GUI labels and their associated data.
    """
    if (
        "connection_lost" not in gui_labels
    ):  # Check if the connection has been lost
        now = datetime.now().strftime("%I:%M:%S %p")
        gui_labels["clock_label"].config(text=f"Connected as of: {now}")
        gui_labels["clock_update"] = gui_labels["clock_label"].after(
            1000, lambda: update_clock(gui_labels)
        )


def start_monitoring(last_data_time, gui_labels):
    """
    Start monitoring the connection and update the GUI accordingly.

    Args:
        last_data_time (list): A list containing the timestamp of the last received data.
        gui_labels (dict): A dictionary containing GUI labels and their associated data.
    """
    monitoring_thread = threading.Thread(
        target=monitor_connection,
        args=(last_data_time, gui_labels),
        daemon=True,
    )
    monitoring_thread.start()


def monitor_connection(last_data_time, gui_labels):
    """
    Monitor the connection and update the GUI if the connection is lost.

    Args:
        last_data_time (list): A list containing the timestamp of the last received data.
        gui_labels (dict): A dictionary containing GUI labels and their associated data.
    """
    while True:
        time.sleep(1)
        if (
            time.time() - last_data_time[0] > 5
            and "connection_lost" not in gui_labels
        ):
            lost_time = datetime.now().strftime("%I:%M:%S %p")
            gui_labels["connection_lost"] = True
            update_gui(gui_labels, False, f"Connection lost at {lost_time}")


def browse_directory(directory_var):
    """
    Open a directory browser dialog and update the directory entry field.

    Args:
        directory_var (StringVar): The variable associated with the directory entry field.
    """
    directory = filedialog.askdirectory()
    if directory:
        if os.path.isdir(directory):
            directory_var.set(directory)
        else:
            messagebox.showerror("Error", "Invalid directory.")


def create_gui():
    """
    Create the graphical user interface (GUI) for the application.

    Returns:
        tuple: A tuple containing the root window, GUI labels, and last data time.
    """
    root = Tk()
    root.title("Syncer")

    frame = Frame(root)  # Frame to center elements before connection
    frame.pack(expand=True)

    # Load and resize the image
    image_path = "/Users/jack/Desktop/Sync/sink_picture.png"
    original_image = Image.open(image_path)
    resized_image = original_image.resize(
        (IMAGE_WIDTH, IMAGE_HEIGHT), Image.Resampling.LANCZOS
    )
    image = ImageTk.PhotoImage(resized_image)

    image_label = Label(frame)
    image_label.pack()

    connect_label = Label(
        frame, text="Connect to the board", font=("Helvetica", 30)
    )
    connect_label.pack()

    clock_label = Label(frame, text="", font=("Helvetica", 20))
    clock_label.pack()

    port_var = StringVar()
    port_combobox = Combobox(frame, textvariable=port_var, width=50)
    port_combobox.pack()

    # Create input fields for filename and directory
    filename_label = Label(frame, text="Filename:")
    filename_label.pack()
    filename_entry = Entry(frame)
    filename_entry.pack()

    directory_label = Label(frame, text="Save Directory:")
    directory_label.pack()
    directory_var = StringVar()
    directory_entry = Entry(frame, textvariable=directory_var, width=50)
    directory_entry.pack()
    browse_button = Button(
        frame, text="Browse", command=lambda: browse_directory(directory_var)
    )
    browse_button.pack()

    connect_button = Button(
        frame,
        text="Connect",
        command=lambda: connect_to_arduino(
            port_combobox.get(),
            last_data_time,
            gui_labels,
            root,
            port_combobox,
            connect_button,
            filename_entry.get(),
            directory_var.get(),
            filename_entry,
            directory_entry,
            browse_button,
        ),
    )
    connect_button.pack()

    ports = comports()
    port_combobox["values"] = [port.device for port in ports]

    last_data_time = [time.time()]
    gui_labels = {
        "time": connect_label,
        "image_label": image_label,
        "image": image,
        "clock_label": clock_label,
        "connect_label": connect_label,
        "clock_update": None,
    }

    return root, gui_labels, last_data_time


def main():
    """Initialize GUI and start application event loop."""
    root, gui_labels, last_data_time = create_gui()
    root.mainloop()
    sys.exit()


if __name__ == "__main__":
    main()
