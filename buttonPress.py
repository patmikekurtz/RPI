from gpizero import Button

myButton = button(23)

def loop:
    while True:
      if myButton.is_pressed:
          print('You pressed me!!')
      if myButton.is_released:
          print('You released me!!')

if __name__ == '__main__':     # Program entrance
    print ('Program is starting...')
    try:
        loop()
    except KeyboardInterrupt:  # Press ctrl-c to end the program.
        print("Ending program")