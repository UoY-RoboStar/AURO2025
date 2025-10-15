
from statemachine import State
from statemachine import StateMachine

def DrivingMachine(StateMachine):
    initial = State('Initial', initial=True)
    driving = State('Forward')

    initial.to(driving, internal=True)

def main():
    print('Hi from week_4.')


if __name__ == '__main__':
    main()
