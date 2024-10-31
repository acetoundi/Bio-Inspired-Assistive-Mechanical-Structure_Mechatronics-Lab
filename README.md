# Educational Exoskeleton

Welcome to the **Educational Exoskeleton** project! This repository contains all the information required to get started with developing firmware for the Tribonix exoskeleton arm using Tribonix Link, setup in the mechatronics lab. This guide will help you set up your development environment and give you a head start for contributing to this project.

The aim is to support students to learn, experiment, and innovate in the field of wearable robotics. With the development board provided and a laptop set up with Arduino IDE or VS Code, you will be able to compile, upload, and test your own code to control the exoskeleton.

Arduino IDE library **autowp-mcp2515**, from **autowp** is required.

## Table of Contents
1. [How to Start Development](#how-to-start-development)
2. [Setup Arduino for Espressif's ESP32S3 Development](#setup-arduino-for-espressifs-esp32s3-development-using-arduino-core)
3. [Setup VS Code for ESP32S3 Development Using ESP-IDF](#setup-vs-code-for-espressifs-esp32s3-development-using-esp-idf)
4. [Getting Started with Git](#getting-started-with-git)
5. [Suggestions for Additional Steps](#suggestions-for-additional-steps)

## How to Start Development

1. **Clone this Repository**: Clone this repository into your working folder on your computer.
2. **Explore Examples**: Navigate into one of the example folders and select the development environment to use.
3. **Edit and Test**: Edit, compile, and test your changes to learn and improve.

## Setup Arduino for Espressif's ESP32S3 Development Using Arduino Core

This section provides step-by-step instructions to set up Arduino for the Tribonix exoskeleton dev board, which uses the ESP32s3.

1. **Install Arduino IDE**: Download and install Arduino IDE from [the official website](https://www.arduino.cc/en/software).
2. **Open the Boards Manager**: Open Arduino IDE, and click the Boards Manager on the left panel.
3. **Install ESP32 Board**: Search for "ESP32 by Espressif Systems," select it, and click **Install**.
4. **Connect the Exoskeleton**: Connect the Tribonix Exoskeleton Link to your device using a USB-C cable. Ensure both the USB port and the cable are capable of data transfer.
5. **Install Drivers**: USB drivers should automatically install when connecting for the first time.
6. **Check Device Connection**: You should be able to see the device connected in the devices list in the top panel of the Arduino IDE. It may be listed as an unknown device.
7. **Select Board**: Select the device, and in the pop-up window, search for and select "ESP32S3 Dev Module".
8. **Ready to Develop**: You are now ready to use the Tribonix Link with your computer.
9. **Upload Code**: Use the "Upload" button to program the board.

## Setup VS Code for Espressif's ESP32S3 Development Using ESP-IDF

This section will guide you in setting up VS Code with the ESP-IDF extension for development.

1. **Install VS Code**: Download and install Visual Studio Code from [the official website](https://code.visualstudio.com/).
2. **Install ESP-IDF Extension**: Open VS Code and navigate to the Extensions menu on the left panel.
3. **Search and Install ESP-IDF**: Search for "ESP-IDF" and select **Express Install**. Choose a location on your computer, and use GitHub as the source.
4. **Check Installation**: After installation, right-click on the bottom panel of VS Code and ensure that "ESP-IDF (Extension)" is enabled.
5. **Connect the Exoskeleton**: Connect the Tribonix Link to your device using a USB-C cable. Ensure the cable and port support data transfer.
6. **Verify USB Drivers**: USB drivers should automatically install if not already present.
7. **Check Device Connection**: You should see the device connected in the bottom VS Code ESP-IDF panel, along with the port used.
8. **Select Target Device**: Use the bottom VS Code ESP-IDF panel to select the target device as "ESP32S3".
9. **Ready to Develop**: You are now ready to use the Tribonix Link with VS Code.
10. **Build and Flash**: Use the "ESP-IDF: Build, Flash and Monitor" button to compile and upload the code.

## Getting Started with Git

To collaborate effectively, we recommend using Git and GitHub for version control. Version control helps keep track of changes, manage collaboration with other developers, and allows you to work efficiently on new features or bug fixes.

### Setting Up Git for Version Control

1. **Download and Install Git**: Download and install Git from [the official website](https://git-scm.com/).
2. **Fork the Repository**: Go to the GitHub repository of this project and fork your own copy of the repository.
3. **Clone the Repository**: Clone your forked repository to your local machine using the following command from Git Bash:
   ```bash
   git clone <repository_link>
   ```
4. **Work on Your Changes**: Navigate into the repository folder and start working on your changes.
5. **Add Changes**: Use `git add <file_name>` or `git add .` to stage your changes for commit.
6. **Commit Your Changes**: Commit your changes with a descriptive message:
   ```bash
   git commit -m "Add descriptive commit message here"
   ```
7. **Push Changes**: Push your changes to update your GitHub repository:
   ```bash
   git push origin main
   ```
8. **Branches**: Read more about branches and try implementing them for new features.

## License
This project is licensed under the MIT License - see the LICENSE file for details.

## Contact
If you have any questions or run into any issues, feel free to reach out to the Tribonix development team or open an issue on the GitHub repository.