from configs import *  # too many variables to import explicitly


def test1():
  return [
    roverMove(ROVER_FORWARD, 60),
    roverMove(ROVER_FORWARD, 60),
  ]

def test2():
  return [
    roverMove(ROVER_FORWARD, 60),
    roverMove(ROVER_RIGHT, 90),
    roverMove(ROVER_FORWARD, 60),
  ]

def test3():
  turn = [
    roverMove(ROVER_FORWARD, 60),
    roverMove(ROVER_RIGHT, 90),
  ]
  return turn * 4 * 3

def test4():
  turn = [roverMove(ROVER_FORWARD, 60) for _ in range(3)] + [roverMove(ROVER_LEFT, 90)]
  return turn * 4 * 2

def test5():
  turn = [roverMove(ROVER_FORWARD, 60) for _ in range(5)] + [roverMove(ROVER_RIGHT, 90)]
  return turn * 4

def test6():
  return [roverMove(ROVER_FORWARD, 60) for _ in range(5)] + [ # 1st straight run
    roverMove(ROVER_RIGHT, 90), # 1st turn
    roverMove(ROVER_FORWARD, 60),
    roverMove(ROVER_RIGHT, 90),
  ] + [roverMove(ROVER_FORWARD, 60) for _ in range(5)] + [ # 2nd straight run
    roverMove(ROVER_LEFT, 90), # 2nd turn
    roverMove(ROVER_FORWARD, 60),
    roverMove(ROVER_LEFT, 90),
  ] + [ # Around 1st obstacle
    roverMove(ROVER_FORWARD, 60),
    roverMove(ROVER_RIGHT, 90),
    roverMove(ROVER_FORWARD, 60),
    roverMove(ROVER_LEFT, 90),
    roverMove(ROVER_FORWARD, 60),
    roverMove(ROVER_FORWARD, 60),
    roverMove(ROVER_LEFT, 90),
    roverMove(ROVER_FORWARD, 60),
    roverMove(ROVER_RIGHT, 90),
  ] + [ # Around 2nd obstacle
    roverMove(ROVER_FORWARD, 60),
    roverMove(ROVER_RIGHT, 90),
    roverMove(ROVER_FORWARD, 60),
    roverMove(ROVER_LEFT, 90),
    roverMove(ROVER_FORWARD, 60),
    roverMove(ROVER_RIGHT, 90),
  ] + [ # 3rd turn
    roverMove(ROVER_FORWARD, 60),
    roverMove(ROVER_RIGHT, 90),
    roverMove(ROVER_FORWARD, 60),
    roverMove(ROVER_RIGHT, 90),
    roverMove(ROVER_FORWARD, 60),
    roverMove(ROVER_LEFT, 90),
  ] + [roverMove(ROVER_FORWARD, 60) for _ in range(4)] + [ # 3rd straight run
    roverMove(ROVER_LEFT, 90), # 4th turn
    roverMove(ROVER_FORWARD, 60),
    roverMove(ROVER_LEFT, 90),
    roverMove(ROVER_FORWARD, 60),
  ] + [ # Around 3rd obstacle
    roverMove(ROVER_FORWARD, 60),
    roverMove(ROVER_RIGHT, 90),
    roverMove(ROVER_FORWARD, 60),
    roverMove(ROVER_LEFT, 90),
    roverMove(ROVER_FORWARD, 60),
    roverMove(ROVER_FORWARD, 60),
    roverMove(ROVER_LEFT, 90),
    roverMove(ROVER_FORWARD, 60),
    roverMove(ROVER_RIGHT, 90),
  ] + [ # 5th turn
    roverMove(ROVER_FORWARD, 60),
    roverMove(ROVER_RIGHT, 90),
    roverMove(ROVER_FORWARD, 60),
    roverMove(ROVER_RIGHT, 90),
  ] + [roverMove(ROVER_FORWARD, 60) for _ in range(4)]

def test7():
  return [roverMove(ROVER_FORWARD, 60) for _ in range(5)] + [ # 1st straight run
    roverMove(ROVER_RIGHT, 90), # 1st turn
    roverMove(ROVER_FORWARD, 60),
    roverMove(ROVER_RIGHT, 90),
  ] + [ # 1st obstacle
    roverMove(ROVER_FORWARD, 60),
    roverMove(ROVER_LEFT, 90),
    roverMove(ROVER_FORWARD, 60),
    roverMove(ROVER_FORWARD, 60),
    roverMove(ROVER_LEFT, 90),
    roverMove(ROVER_FORWARD, 60),
    roverMove(ROVER_RIGHT, 90),
  ] + [ # 2nd turn
    roverMove(ROVER_FORWARD, 60),
    roverMove(ROVER_FORWARD, 60),
    roverMove(ROVER_RIGHT, 90),
  ] + [roverMove(ROVER_FORWARD, 60) for _ in range(4)] + [ # 2nd straight run
    roverMove(ROVER_RIGHT, 90), # 2nd obstacle
    roverMove(ROVER_FORWARD, 60),
    roverMove(ROVER_LEFT, 90),
    roverMove(ROVER_FORWARD, 60),
    roverMove(ROVER_RIGHT, 90),
  ] + [roverMove(ROVER_FORWARD, 60) for _ in range(4)] + [ # 3rd straight run
    roverMove(ROVER_RIGHT, 90) # 3rd turn
  ] + [roverMove(ROVER_FORWARD, 60) for _ in range(3)] + [ # 4th straight run
    roverMove(ROVER_RIGHT, 90), # 4th turn
    roverMove(ROVER_FORWARD, 60),
    roverMove(ROVER_FORWARD, 60),
  ] + [ # 3rd obstacle
    roverMove(ROVER_LEFT, 90),
    roverMove(ROVER_FORWARD, 60),
    roverMove(ROVER_RIGHT, 90),
    roverMove(ROVER_FORWARD, 60),
    roverMove(ROVER_FORWARD, 60),
    roverMove(ROVER_RIGHT, 90),
    roverMove(ROVER_FORWARD, 60),
    roverMove(ROVER_FORWARD, 60),
    roverMove(ROVER_RIGHT, 90),
    roverMove(ROVER_FORWARD, 60),
    roverMove(ROVER_FORWARD, 60),
  ] + [ # 4th obstacle
    roverMove(ROVER_LEFT, 90),
    roverMove(ROVER_FORWARD, 60),
    roverMove(ROVER_RIGHT, 90),
    roverMove(ROVER_FORWARD, 60),
  ]


if __name__ == "__main__":
  import pdb; pdb.set_trace()
