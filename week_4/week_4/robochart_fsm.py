
from pydantic import BaseModel, ConfigDict
from enum import Enum
from typing import TypeVar, Generic

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
        self.iterator = None
    
    def step_clocks(self):
        ''' Step all clocks in the context. '''
        if self.context is not None:
            for field_name, field_info in self.context.model_fields.items():
                value = getattr(self.context, field_name)
                if isinstance(value, Clock):
                    self.logger.debug(f"Stepping clock '{field_name}'.")
                    value.step()
        return

    def name(self):
        return self.__class__.__name__

    def get_context(self):
        return self.context

    def execute(self, timed = True, **params):
        try:
            if self.iterator is None:
                self.iterator = self.initial()
                return next(self.iterator)
            else:
                if timed:
                    self.step_clocks()
                return self.iterator.send(timed)
        except StopIteration:
            self.logger.info(f"Execution of machine '{self.name()}' has finished.")
            pass

    def initial(self):
        raise NotImplementedError(f"Initial junction of machine '{self.name()}' has not been implemented.")

    def enter(self):

        result = self.entry_action()
        if result is not None:
            yield from result

        # Enter substate (if any)
        while self.execute() is not MachineStatus.ENTERED:
            self.logger.info(f"Waiting to enter machine '{self.name()}'")
            yield

    def exit(self):
        # Enter substate (if any)
        result = self.exit_action()

        while result is not MachineStatus.FINISHED:
            self.logger.info(f"Waiting to exit machine '{self.name()}'")
            yield

    def exit_action(self):
        return MachineStatus.FINISHED

    def entry_action(self):
        return None

    def during_action(self):
        return None
