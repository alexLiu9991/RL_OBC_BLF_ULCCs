"""
Copyright 1996-2022 Cyberbotics Ltd.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

Description: Animate water flow
"""

from controller import Robot, Display

TIME_STEP = 32
WATER_TILE_HEIGHT = 512

def step(robot):
    if robot.step(TIME_STEP) == -1:
        robot.cleanup()
        exit(0)

def main():
    # Initialize the robot
    robot = Robot()
    
    # Get the display device
    water_display = robot.getDevice("water display")
    
    # Load the image
    water_image = water_display.imageLoad("water_flow.png")
    
    # Main loop
    counter = 0
    increment = 0.4
    number_of_images = WATER_TILE_HEIGHT // increment
    
    while True:
        # counter = counter % number_of_images
        # shift = -WATER_TILE_HEIGHT + counter * increment
        # water_display.imagePaste(water_image, shift, shift, True)
        # counter += 1
        step(robot)

if __name__ == "__main__":
    main()