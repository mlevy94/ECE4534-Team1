from configs import *  # too many variables to import explicitly


def test1():
  return [
    roverMove(ROVER_FORWARD, 60, COORDINATOR),
    roverMove(ROVER_FORWARD, 60, COORDINATOR),
  ]

def test2():
  return [
    roverMove(ROVER_FORWARD, 60, COORDINATOR),
    roverMove(ROVER_RIGHT, 90, COORDINATOR),
    roverMove(ROVER_FORWARD, 60, COORDINATOR),
  ]

def test3():
  turn = [
    roverMove(ROVER_FORWARD, 60, COORDINATOR),
    roverMove(ROVER_RIGHT, 90, COORDINATOR),
  ]
  return turn * 4 * 3

def test4():
  turn = [roverMove(ROVER_FORWARD, 60, COORDINATOR) for _ in range(3)] + [roverMove(ROVER_LEFT, 90, COORDINATOR)]
  return turn * 4 * 2

def test5():
  turn = [roverMove(ROVER_FORWARD, 60, COORDINATOR) for _ in range(5)] + [roverMove(ROVER_RIGHT, 90, COORDINATOR)]
  return turn * 4

def test6():
  return [roverMove(ROVER_FORWARD, 60, COORDINATOR) for _ in range(5)] + [ # 1st straight run
    roverMove(ROVER_RIGHT, 90, COORDINATOR), # 1st turn
    roverMove(ROVER_FORWARD, 60, COORDINATOR),
    roverMove(ROVER_RIGHT, 90, COORDINATOR),
  ] + [roverMove(ROVER_FORWARD, 60, COORDINATOR) for _ in range(5)] + [ # 2nd straight run
    roverMove(ROVER_LEFT, 90, COORDINATOR), # 2nd turn
    roverMove(ROVER_FORWARD, 60, COORDINATOR),
    roverMove(ROVER_LEFT, 90, COORDINATOR),
  ] + [ # Around 1st obstacle
    roverMove(ROVER_FORWARD, 60, COORDINATOR),
    roverMove(ROVER_RIGHT, 90, COORDINATOR),
    roverMove(ROVER_FORWARD, 60, COORDINATOR),
    roverMove(ROVER_LEFT, 90, COORDINATOR),
    roverMove(ROVER_FORWARD, 60, COORDINATOR),
    roverMove(ROVER_FORWARD, 60, COORDINATOR),
    roverMove(ROVER_LEFT, 90, COORDINATOR),
    roverMove(ROVER_FORWARD, 60, COORDINATOR),
    roverMove(ROVER_RIGHT, 90, COORDINATOR),
  ] + [ # Around 2nd obstacle
    roverMove(ROVER_FORWARD, 60, COORDINATOR),
    roverMove(ROVER_RIGHT, 90, COORDINATOR),
    roverMove(ROVER_FORWARD, 60, COORDINATOR),
    roverMove(ROVER_LEFT, 90, COORDINATOR),
    roverMove(ROVER_FORWARD, 60, COORDINATOR),
    roverMove(ROVER_RIGHT, 90, COORDINATOR),
  ] + [ # 3rd turn
    roverMove(ROVER_FORWARD, 60, COORDINATOR),
    roverMove(ROVER_RIGHT, 90, COORDINATOR),
    roverMove(ROVER_FORWARD, 60, COORDINATOR),
    roverMove(ROVER_RIGHT, 90, COORDINATOR),
    roverMove(ROVER_FORWARD, 60, COORDINATOR),
    roverMove(ROVER_LEFT, 90, COORDINATOR),
  ] + [roverMove(ROVER_FORWARD, 60, COORDINATOR) for _ in range(4)] + [ # 3rd straight run
    roverMove(ROVER_LEFT, 90, COORDINATOR), # 4th turn
    roverMove(ROVER_FORWARD, 60, COORDINATOR),
    roverMove(ROVER_LEFT, 90, COORDINATOR),
    roverMove(ROVER_FORWARD, 60, COORDINATOR),
  ] + [ # Around 3rd obstacle
    roverMove(ROVER_FORWARD, 60, COORDINATOR),
    roverMove(ROVER_RIGHT, 90, COORDINATOR),
    roverMove(ROVER_FORWARD, 60, COORDINATOR),
    roverMove(ROVER_LEFT, 90, COORDINATOR),
    roverMove(ROVER_FORWARD, 60, COORDINATOR),
    roverMove(ROVER_FORWARD, 60, COORDINATOR),
    roverMove(ROVER_LEFT, 90, COORDINATOR),
    roverMove(ROVER_FORWARD, 60, COORDINATOR),
    roverMove(ROVER_RIGHT, 90, COORDINATOR),
  ] + [ # 5th turn
    roverMove(ROVER_FORWARD, 60, COORDINATOR),
    roverMove(ROVER_RIGHT, 90, COORDINATOR),
    roverMove(ROVER_FORWARD, 60, COORDINATOR),
    roverMove(ROVER_RIGHT, 90, COORDINATOR),
  ] + [roverMove(ROVER_FORWARD, 60, COORDINATOR) for _ in range(4)]

def test7():
  pass


if __name__ == "__main__":
  import pdb; pdb.set_trace()
