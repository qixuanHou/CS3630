# STATES

# start - intial state doesn't know anything has to look for ball

# traveling - found the ball and has to keep track of it and travel to it

# end - at the ball and has to tap it

# ~pause - stop cozmo for a sec

class State():
    def __init__(self):
        self.states = ["START", "TRAVELING", "END", "PAUSE"]
        self.cur = "START"

    def next(lost=False):
        if lost:
            self.cur = "PAUSE"
        else:
            if self.cur = "START":
                self.cur = "TRAVELING"

            if self.cur = "TRAVELING":
                self.cur = "END"

            if self.cur = "END":
                self.cur = "END"

            if self.cur = "PAUSE":
                self.cur = "START"

    def set(s):
        if s in self.states:
            state.cur = s
        else:
            return False
        return True


state = State()
if state.cur == "START":
    #spin around and search for ball
    await robot.drive_wheels(25,-25)
    if ballFound:
        state.next()

if state.cur == "TRAVELING":
    #move towards ball
    if ballLost:
        state.next(True)
    elif atBall:
        state.next()

if state.cur == "END":
    #tap ball

if state.cur == "PAUSE":
    #pause for a moment
    await robot.drive_wheels(0, 0, 0.05)
    state.next()
