from selfdrive.body.bodyd import BodyD
from selfdrive.car.honda.bodyvalues import CAR

# test that we can init with every supported car


SUPPORTED_HONDA = [CAR.ACCORD,
                  CAR.CIVIC_BOSCH,
                  CAR.CIVIC_BOSCH_DIESEL]


def main():
  for car in SUPPORTED_HONDA:
    print("Testing", car)
    b = BodyD(cache=car)
    assert b.body is not None, ("body not loaded for", car)


if __name__ == '__main__':
  main()
