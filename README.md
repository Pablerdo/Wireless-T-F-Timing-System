# Wireless-T-F-Timing-System

During my Track and Field training sessions, my coach often expressed his desire to have a more precise way of measuring an athlete's performance. However, living in a third-world-country means that shipping from the US is expensive and sports clubs and educational institutions do not have precise timing systems as a funding priority.

I had been working on line-follower robots with Arduino for some time, so I realized that an affordable, wireless and precise laser gate timing system made of easy to find components should be possible. Accounting for the different prices across the world, absolutely all of the components and parts needed for the complete system should cost around 100USD. This is a major price difference with other systems, considering that it has a very decent precision and has a wireless range that exceeds almost any Track and Field oval in the world.

The system consists of two finish line modules and one start line module.

In the finish line, you will have an Arduino UNO, nRF24L01+ Transceiver, LED button pad, photoresistors, LEDs, plastic enclosure, and tripod together with my Finish Line PCB. This module acts as the master, from the LED button pad you will decide when to start the timer.

On the opposite side of the track, you will have a laser that is aligned with the photoresistors. You can either buy a Keyes laser module, a 3.3V regulator, a 9V battery, and my Laser Module PCB, or you can provide your own laser pointer. What matters is that the laser is in a steady position and provides enough light.

In the start line, you will have an Arduino UNO, nRF24L01+ Transceiver, speaker, LEDs, plastic enclosure, and my Start Line PCB. This module is in constant communication with the master and uses an LED to inform the user that it is still connected. When measuring time, the LED will change color.

A more detailed overview of this project can be found at:

https://create.arduino.cc/projecthub/Pablerdo/wireless-laser-gate-timing-system-for-track-and-field-ba8cd9?ref=search&ref_id=track%20and%20field%20&offset=2

https://www.hackster.io/Pablerdo/wireless-laser-gate-timing-system-for-track-and-field-ba8cd9
