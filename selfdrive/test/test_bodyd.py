from selfdrive.body import BodyCan

def main():
  b = BodyCan()
  b.send("unlock")
  # print(type(b.bus))

if __name__ == '__main__':
  main()
