#!/usr/bin/env python

from jsk_rviz_plugins.msg import *
from std_msgs.msg import ColorRGBA
import rospy
import math
rospy.init_node("aaahoge")

pub = rospy.Publisher("text_sample", OverlayText)
counter = 0
r = rospy.Rate(2)

lyrics = """I read the news today oh boy 
About a lucky man who made the grade 
And though the news was rather sad 
Well I just had to laugh 

I saw the photograph. 
He blew his mind out in a car 
He didn't notice that the lights had changed 
A crowd of people stood and stared 
They'd seen his face before 
Nobody was really sure 
If he was from the House of Lords. 

I saw a film today oh boy 
The English army had just won the war 
A crowd of people turned away 
But I just had to look 
Having read the book 
I'd love to turn you on. 

Woke up, fell out of bed, 
Dragged a comb across my head 
Found my way downstairs and drank a cup, 
And looking up I noticed I was late. 
Found my coat and grabbed my hat 
Made the bus in seconds flat 
Found my way upstairs and had a smoke, 
Somebody spoke and I went into a dream. 

I read the news today oh boy 
Four thousand holes in Blackburn, Lancashire 
And though the holes were rather small 
They had to count them all 
Now they know how many holes it takes to fill the Albert Hall. 
I'd love to turn you on"""
import random
while not rospy.is_shutdown():
  text = OverlayText()
  theta = counter % 255 / 255.0
  text.width = 400
  text.height = 600
  #text.height = 600
  text.left = 10
  text.top = 10
  text.text_size = 12
  text.line_width = 2
  text.text = "\n".join(lyrics.split("\n")[counter:])
  text.fg_color = ColorRGBA(25 / 255.0, 1.0, 240.0 / 255.0, 1.0)
  text.bg_color = ColorRGBA(0.0, 0.0, 0.0, 0.2)
  pub.publish(text)
  counter = counter + 1
  if (counter > len(lyrics.split("\n"))):
    counter = 0
  r.sleep()

