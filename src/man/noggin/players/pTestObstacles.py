
from . import SoccerFSA
from . import TestObstaclesStates

class SoccerPlayer(SoccerFSA.SoccerFSA):
    def __init__(self, brain):
        SoccerFSA.SoccerFSA.__init__(self, brain)
        self.addStates(TestObstaclesStates)
        self.setName('pTestObstacles')
