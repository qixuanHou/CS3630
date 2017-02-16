# STATES

# start - intial state doesn't know anything has to look for ball

# traveling - found the ball and has to keep track of it and travel to it

# end - at the ball and has to tap it

# ~pause - stop cozmo for a sec
import subprocess
class State():
    def __init__(self):
        self.states = ["START", "TRAVELING", "END", "PAUSE"]
        self.cur = "START"
        print("Intialized to", self.cur, "state")

    def isCurState(self, check):
        return check == self.cur

    def next(self, lost=False):
        print("Current State:", self.cur)
        if lost:
            self.cur = "PAUSE"
        else:
            if self.cur == "START":
                self.cur = "TRAVELING"

            elif self.cur == "TRAVELING":
                self.cur = "END"

            elif self.cur == "END":
                self.cur = "TRAVELING"

            elif self.cur == "PAUSE":
                self.cur = "START"

            print("Switching to state:", self.cur)
            # subprocess.run(["paplay", "./blip.ogg"])
    def set(self, s):
        if s in self.states:
            state.cur = s
        else:
            return False
        return True
