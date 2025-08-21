# Automatic Pet Feeder

This project is a 3D-printed automatic pet feeder powered by an ESP32-C3 Supermini. It features persistent memory for storing feeding schedules, a menu-driven user interface displayed on an OLED screen, and user interaction via a rotary encoder.

## Features

*   **3D-printed parts:** All structural components of the feeder are designed to be 3D-printed.
*   **Persistent memory:** Feeding schedules and settings are stored in the ESP32's non-volatile memory, ensuring they persist even after power loss.
*   **Menu-driven UI:** Easy configuration and control through an intuitive menu system displayed on an OLED screen, navigated with a rotary encoder.
*   **Deep sleep for power efficiency:** The ESP32 utilizes deep sleep mode to conserve power between feeding times.
*   **Programmable feeding times:** Users can set multiple specific times for food dispensing.

## Components Used

*   **Microcontroller:** ESP32-C3 Supermini
*   **Stepper Motor:** 28BYJ-48 Geared Stepper Motor
*   **Stepper Motor Driver:** (e.g., ULN2003 or similar, required for the 28BYJ-48 motor)
*   **Display:** OLED Display (SSD1306, 128x64 pixels)
*   **User Input:** Rotary Encoder with Push Button
*   **Real-Time Clock (RTC):** DS1307 RTC Module
*   **Power Supply:** (e.g., 5V DC power adapter)
*   **Wiring:** Jumper wires, breadboard (optional for prototyping)

## Pictures

![Completed](pictures/completed.jpg)

## STLs

The STLs for the 3D-printed parts are located in the `STLs` directory.

## Code

The Arduino code for the feeder is located in the `catfeederPersistentMem.ino` file.