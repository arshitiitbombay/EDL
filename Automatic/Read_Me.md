# Control ideas
## How to push the bot out of the box

Push one link out at a time by clicking P. Makes it easy to click and explore.

## Sequence:
1. Get coordinates from laptop. Laptop will calculate which motors to actuate based on mapping, and send this to master RPi.
2. Divide total ticks by 10 and call this t. Move forward dz, then bend by t, then again forward by dz and so on.
3. For the next link: store front (earlier) link's encoder values, and move it in the same way. And for the new link, treat it as the same as old one.
4. For bringing the robot back, we have stored the sequence, and for last link, we send all values as 0.

## Functionalities:
1. Limit switch trigger: calculate and move accordingly (just that one link).
2. Press "t": the frontmost link just explores based on mouse input, does not move forward.
3. Press "x": One link backward.
