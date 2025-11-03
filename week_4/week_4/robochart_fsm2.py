
from pydantic import BaseModel, ConfigDict
from enum import Enum
from typing import TypeVar, Generic, Iterator, Callable
import functools
import inspect
import types

T = TypeVar("T")

class MachineStatus(Enum):
    ENTERING = 0
    ENTERED = 1 
    FINISHED = 2

class Clock:
    
    def __init__(self, timer_period: int, time_unit: int):
        self.timer_period = timer_period
        self.time_unit = time_unit
        self.count = 0

    def step(self):
        self.count += 1

    def reset(self):
        self.count = 0

    def since(self):
        return self.timer_period * self.count / self.time_unit

class Event:
    def __init__(self):
        self.is_set = False

    def set(self):
        self.is_set = True

    def clear(self):
        self.is_set = False

    def is_triggered(self):
        return self.is_set

class TypedEvent(Generic[T], Event):

    def __init__(self, value: T = None):
        self.value = value
        super().__init__()
        
    def set(self, value: T = None):
        self.value = value
        super().set()
        
    def clear(self):
        self.value = None
        super().clear()

class TypedOutputEvent(Generic[T], Event):

    def __init__(self, value: T = None, impl: Callable[[T],None] = None):
        self.value = value
        self.impl = impl
        super().__init__()
        
    def set(self, value: T = None):
        self.value = value
        super().set()
        if self.impl is not None:
            self.impl(self.value)
            super().clear
        
    def clear(self):
        self.value = None
        super().clear()

class Context(BaseModel):
    parent: "Context | None" = None

    def __init__(self, parent = None, **data):
        super().__init__(**data)
        self.parent = parent

    def __getattr__(self, attr):
        ''' Recursive attribute lookup in parent contexts. '''
        if self.parent and hasattr(self.parent, attr):
            return getattr(self.parent, attr)
        raise AttributeError(f"Attribute {attr!r} not found in hierarchy. Did you forget to declare {attr!r} in a Context?")

    def model_dump_safe(self):
        '''Safe recursive dump excluding parent links.'''
        return self.model_dump(exclude={"parent"})

    class Config:
        arbitrary_types_allowed = True

class Machine:

    def __init__(self, logger, timer_period, time_unit = 1, context: Context = Context()):
        self.logger = logger
        self.context = context
        self.timer_period = timer_period
        self.time_unit = time_unit
        self.node = None
        self.node_prime = True
        self.gen_started = False
    
    def step_clocks(self):
        return

    def name(self):
        return self.__class__.__name__

    def get_context(self):
        return self.context

    def execute(self, timed = True, exit = False, **params):

        if timed:
            self.step_clocks()

        while True:
            try:              
                if self.node is None:
                    self.node = self.initial
                
                # It's a method for a node/state
                if callable(self.node):
                    # We call the method
                    self.node = self.node()
                    self.gen_exec = False

                    # If there is no subsequent state, the machine has terminated
                    # so return an empty list.
                    if self.node is None:
                        return []

                # It's a generator
                elif isinstance(self.node, types.GeneratorType):
                    # Unlike a top-level state machine, we delegate execution of the node by yielding from it,
                    # rather than returning to the callee.
                    if not self.gen_exec:
                        self.gen_exec = True
                        self.node = yield from self.node                        
                    else:
                        self.node = yield from self.node.send((timed,exit))

            
            except StopIteration as e:
                # If the node stops, then it means all work so far has been done,
                # potentially because there's been a state change.
                self.node = e.value

        return 

    def initial(self, **params):
        raise NotImplementedError(f"Initial junction of machine '{self.name()}' has not been implemented.")

    def enter(self):

        result = self.entry_action()
        if result is not None:
            yield from result

        # Enter substate (if any)
        while self.execute() is not MachineStatus.ENTERED:
            self.logger.debug(f"Waiting to enter machine '{self.name()}'")
            yield

    def exit(self):

        # Exit substate (if any)
        while self.execute(exit = True) is not MachineStatus.FINISHED:
            self.logger.debug(f"Waiting to exit machine '{self.name()}'")
            yield

        # Enter substate (if any)
        result = self.exit_action()
        if result is not None:
            yield from result

    def exit_action(self):
        return None

    def entry_action(self):
        return None

    def during_action(self):
        return None

class StateMachine(Machine):

    def __init__(self, logger, timer_period, time_unit = 1, context: Context = Context()):
        super().__init__(logger, timer_period, time_unit, context)
        self.iter = None
        self.gen_exec = False

    def step_clocks(self):
        ''' Step all clocks in the context. '''
        if self.context is not None:
            for field_name, field_info in self.context.model_fields.items():
                value = getattr(self.context, field_name)
                if isinstance(value, Clock):
                    self.logger.debug(f"Stepping clock '{field_name}'.")
                    value.step()

    def execute(self, timed = True, exit = False, **params):

        if timed:
            self.step_clocks()

        self.logger.debug(f"Execute (timed: {timed}, exit: {exit}) of {self.name} called")

        while True:
            try:
                # An empty node
                if self.node is None:
                    self.node = self.initial
                
                # It's a method for a state
                if callable(self.node):
                    # We call the method
                    self.node = self.node()
                    self.gen_exec = False

                    # If there is no subsequent state, the machine has terminated
                    # so return that it has finished.
                    if self.node is None:
                        return MachineStatus.FINISHED

                # It's a generator
                elif isinstance(self.node, types.GeneratorType):
                    # We step the generator (till the next yield), returning to the callee afterwards
                    if not self.gen_exec:
                        self.gen_exec = True
                        return next(self.node)
                    else:
                        return self.node.send((timed, exit))
                else:
                    self.gen_exec = False
                    return self.node
            
            except StopIteration as e:
                # If the generator stops, then it means all work so far has been done,
                # potentially because there's been a state change.
                self.node = e.value

def Generator(func):
    def wrapper(self, *args, **kwargs):
        # Call the original function
        result = func(self, *args, **kwargs)
        # Yield all if it's already a generator
        if hasattr(result, '__iter__') and not isinstance(result, (str, bytes)):
            self.logger.info('Delegating because an iter exists')
            yield from result

        self.node = None
        # Then yield once more at the end
        yield MachineStatus.FINISHED
    return wrapper

def State(_func=None, *, entry=None):
    '''
    Decorator for FSM state methods.
    Handles:
      - Optional entry action (executed once before loop)
      - Infinite loop of condition checks
      - Auto 'yield' when no transition
      - Returns callable state when transition occurs
    '''

    def decorator(func):
        @functools.wraps(func)
        def wrapper(self, *args, **kwargs):
            # --- Entry Action ---
            if entry is not None:
                # entry can be a callable or string name
                if callable(entry):
                    entry(self)
                elif isinstance(entry, str):
                    # call named method on self if provided as string
                    getattr(self, entry)()
                else:
                    raise TypeError("entry must be a callable or method name string")

            # --- Main Loop ---
            while True:
                result = func(self, *args, **kwargs)

                if callable(result):
                    return result
                elif isinstance(result, types.GeneratorType):
                    yield from result
                else:
                    yield # from self.Yielding()

        return wrapper

    # Support both @auto_loop and @auto_loop(...)
    if _func is None:
        return decorator
    else:
        return decorator(_func)