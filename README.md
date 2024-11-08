# Bird Repelling Prototype - AI-Powered Bird Repellent System

Author: [Amenyedzi, D. K.; Vodacek, A.; Kazeneza, M.; Mwaisekwa, I. I.; Nzanywayingoma, F.;
Nsengiyumva, P.; Bamurigire, P.; and Ndashimye, E.]
Date: [2024]

## Introduction
Welcome to the Pest Bird Repelling Prototype project repository. This project leverages edge AI
platform provided by Edge Impulse to detect and repel pest birds by identifying their calls and
triggering a response that emits sounds designed to scare them away. The ultimate aim is to
assist farmers (especially holder farmers in low-income countries) in protecting their crops from
pest bird damage to increase agricultural yield.

## This repository contains
* Code and model for bird sound detection
* Sample data for testing the prototype
* Instructions for setting up the prototype on compatible microcontrollers

The model is an evolving one to make the prototype more effective. We will continue to update
this repository, incorporating new pest bird species and refining detection capabilities. The
current version of the model and data are available for download via
https://studio.edgeimpulse.com/public/491069/latest for those with Edge Impulse account. For
those without Edge Impulse account, Click [Here](https://drive.google.com/drive/folders/1ea5hSRjF4oRSP3Ecxe8ZgFzBOGLZrBLS)

## Project Overview
With tinyML model, this system identifies pest bird species by recognizing their calls or songs
and upon detection, triggers a sound loud enough to deter such birds. This project is designed
as an edge-AI solution, meaning it runs directly on hardware at the source of the problem,
allowing for fast response times and offline operation in field conditions. Future versions will
include additional bird species and advanced deterrent features.

## Requirements
This project requires the following hardware and software:
### Hardware:
Arduino Nano 33 BLE Sense or XIAO ESP32S3 Sense (or similar microcontroller),
Single channel relay module, SIM800L GSM module, GPS, Lithium-ion batteries, and optional
(buzzer and limit switch depending on if you are interested in implementing the security).
### Software: 
Arduino IDE with required libraries _(instructions below):_

1. Download the model:
Download the zipped library file from Edge Impulse via via
_https://studio.edgeimpulse.com/public/491069/latest_ or through this _[link](https://drive.google.com/drive/folders/1ea5hSRjF4oRSP3Ecxe8ZgFzBOGLZrBLS)._
2. Import to Arduino IDE:
Open Arduino IDE on your computer.
Go to Sketch -> Include Library -> Add .ZIP Library... and select the downloaded library.
3. Upload Code:
Connect your microcontroller to your computer.
Upload the bird repelling code provided in this repository to your device.
4. Serial Monitor:
Open the Serial Monitor in Arduino IDE to observe model detections in real-time.
Play any of the pre-trained bird sounds to test the system’s response.

### Running the Model
Once setup is complete:
- Ensure your microcontroller is powered on and connected to the computer.
- Observe detection events in the Serial Monitor as it responds to the pest bird calls listed
in the model.
The system will output a scare sound when a pest bird call is detected, helping to repel
birds from your crops.

## Model and Data
The latest model and dataset are accessible via. This model is trained to recognize specific pest
bird calls. We will add more species to enhance its functionality and broaden its application.
Future Development
We plan to expand this project by:
- Training the model on additional pest bird species
- Optimizing response sounds for greater effectiveness
- Explore integration with other microcontrollers for diverse applications
  
## Contributing
We welcome contributions to enhance and improve this prototype. Feel free to fork the
repository, submit issues, or propose new features.

## License
This project is licensed under [Your License Here]. See the LICENSE file for details.
