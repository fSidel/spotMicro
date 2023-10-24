#!/usr/bin/env python

"""
This class listens the coordinates topic of detected objects and the video
capture topic to render a real time video of the detected objects with 
boundaries drawn on top of the detection.
This class slows down the already slow detection process so only initialize
the node for debug purposes when on a virtualized development environment
"""